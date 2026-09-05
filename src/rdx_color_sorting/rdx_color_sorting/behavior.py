"""Hardware-independent sorting state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable, Mapping


@dataclass(frozen=True)
class Detection:
    """A color target measured in normalized image coordinates."""

    color: str
    x_error: float
    area_ratio: float
    range_m: float | None = None
    shape: str = "cuboid"


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose in the controller's configured reference frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Zone:
    """Sorting-zone center in the configured coordinate convention."""

    x: float
    y: float


@dataclass(frozen=True)
class VelocityCommand:
    """Holonomic base velocity command."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class SortState(str, Enum):
    SEARCH = "search"
    ALIGN = "align"
    APPROACH = "approach"
    PUSH = "push"
    RELEASE = "release"
    RETURN = "return"
    COMPLETE = "complete"
    FAULT = "fault"


@dataclass(frozen=True)
class ControllerConfig:
    search_angular_speed: float = 0.18
    maximum_search_duration: float = 40.0
    search_timeout_is_fault: bool = False
    finish_when_no_target_duration: float = 0.0
    minimum_detection_duration: float = 0.25
    align_kp: float = 0.45
    max_align_speed: float = 0.25
    align_tolerance: float = 0.08
    approach_speed: float = 0.08
    minimum_approach_speed: float = 0.03
    approach_range_kp: float = 0.40
    contact_distance: float = 0.24
    contact_area_ratio: float = 0.12
    target_lost_timeout: float = 0.6
    push_speed: float = 0.08
    maximum_push_duration: float = 20.0
    push_target_lost_timeout: float = 0.0
    push_kp: float = 0.7
    yaw_kp: float = 0.8
    max_yaw_speed: float = 0.20
    zone_tolerance: float = 0.12
    release_speed: float = 0.06
    release_duration: float = 1.0
    delivery_confirmation_timeout: float = 0.0
    delivery_confirmation_duration: float = 0.25
    delivery_maximum_area_ratio: float = 0.15
    delivery_maximum_area_scale: float = 0.85
    delivery_maximum_x_error: float = 0.35
    return_speed: float = 0.10
    return_kp: float = 0.7
    return_tolerance: float = 0.12
    return_corridor_offset: float = 0.80
    return_corridor_margin: float = 0.25
    maximum_return_duration: float = 35.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def transform_pose(transform: Pose2D, pose: Pose2D) -> Pose2D:
    """Apply a planar frame transform to a pose."""
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    return Pose2D(
        transform.x + cos_yaw * pose.x - sin_yaw * pose.y,
        transform.y + sin_yaw * pose.x + cos_yaw * pose.y,
        normalize_angle(transform.yaw + pose.yaw),
    )


def frame_transform(target_pose: Pose2D, source_pose: Pose2D) -> Pose2D:
    """Find the transform that maps source_pose onto target_pose."""
    yaw = normalize_angle(target_pose.yaw - source_pose.yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return Pose2D(
        target_pose.x - (cos_yaw * source_pose.x - sin_yaw * source_pose.y),
        target_pose.y - (sin_yaw * source_pose.x + cos_yaw * source_pose.y),
        yaw,
    )


class SortController:
    """Select, approach, and move each configured color to its zone."""

    def __init__(
        self,
        zones: Mapping[str, Zone],
        targets_per_color: Mapping[str, int],
        color_order: Iterable[str],
        config: ControllerConfig | None = None,
        zones_relative_to_origin: bool = True,
        external_return_enabled: bool = False,
    ) -> None:
        self.config = config or ControllerConfig()
        self.zones = dict(zones)
        self.remaining = dict(targets_per_color)
        self.color_order = tuple(color_order)
        self.zones_relative_to_origin = zones_relative_to_origin
        self.external_return_enabled = external_return_enabled
        if not self.color_order:
            raise ValueError("color_order must not be empty")
        if set(self.color_order) != set(self.zones):
            raise ValueError("color_order and zones must contain the same colors")
        if any(self.remaining.get(color, 0) < -1 for color in self.color_order):
            raise ValueError("target counts must be -1 or non-negative")
        self.processed = {color: 0 for color in self.color_order}

        self.state = SortState.SEARCH
        self.fault_reason: str | None = None
        self._color_index = 0
        self._origin: Pose2D | None = None
        self._last_seen_at: float | None = None
        self._search_started_at: float | None = None
        self._no_target_started_at: float | None = None
        self._candidate_color: str | None = None
        self._candidate_started_at: float | None = None
        self._push_started_at: float | None = None
        self._push_target_last_seen_at: float | None = None
        self._contact_area_ratio: float | None = None
        self._release_started_at: float | None = None
        self._delivery_candidate_started_at: float | None = None
        self._return_started_at: float | None = None
        self._return_waypoints: tuple[tuple[float, float], ...] = ()
        self._return_waypoint_index = 0
        self._push_heading = 0.0
        self._select_next_color()

    @property
    def current_color(self) -> str | None:
        if self.state == SortState.COMPLETE:
            return None
        return self.color_order[self._color_index]

    def set_origin(self, pose: Pose2D) -> None:
        if self._origin is None:
            self._origin = pose

    @property
    def origin(self) -> Pose2D | None:
        return self._origin

    def complete_external_return(self, now: float) -> None:
        if self.state != SortState.RETURN or not self.external_return_enabled:
            raise RuntimeError("external return is not active")
        self._complete_return(now)

    def stop_with_fault(self, reason: str) -> VelocityCommand:
        self.state = SortState.FAULT
        self.fault_reason = reason
        return VelocityCommand()

    def update(
        self,
        now: float,
        detections: Iterable[Detection],
        pose: Pose2D | None,
    ) -> VelocityCommand:
        if self.state in (SortState.COMPLETE, SortState.FAULT):
            return VelocityCommand()
        if pose is not None:
            self.set_origin(pose)

        detections = tuple(detections)
        color = self.current_color

        if self.state == SortState.SEARCH:
            if self._search_started_at is None:
                self._search_started_at = now
            search_duration = now - self._search_started_at

            detection = self._largest_available_detection(detections)
            if detection is None:
                self._candidate_color = None
                self._candidate_started_at = None
                if self._no_target_started_at is None:
                    self._no_target_started_at = now
                if (
                    self.config.finish_when_no_target_duration > 0.0
                    and now - self._no_target_started_at
                    >= self.config.finish_when_no_target_duration
                ):
                    self.state = SortState.COMPLETE
                    return VelocityCommand()
                if (
                    self.config.maximum_search_duration > 0.0
                    and search_duration >= self.config.maximum_search_duration
                ):
                    if self.config.search_timeout_is_fault:
                        return self.stop_with_fault("search timeout")
                    self._search_started_at = now
                return VelocityCommand(angular_z=self.config.search_angular_speed)
            self._no_target_started_at = None
            if self._candidate_color != detection.color:
                self._candidate_color = detection.color
                self._candidate_started_at = now
            if (
                self._candidate_started_at is None
                or now - self._candidate_started_at
                < self.config.minimum_detection_duration
            ):
                return VelocityCommand()
            self._color_index = self.color_order.index(detection.color)
            self._search_started_at = None
            self._no_target_started_at = None
            self._candidate_color = None
            self._candidate_started_at = None
            self._last_seen_at = now
            self.state = SortState.ALIGN
            return VelocityCommand()

        if self.state in (SortState.ALIGN, SortState.APPROACH):
            detection = self._largest_detection(detections, color)
            if detection is None:
                if (
                    self._last_seen_at is None
                    or now - self._last_seen_at > self.config.target_lost_timeout
                ):
                    self.state = SortState.SEARCH
                    self._search_started_at = now
                    self._no_target_started_at = None
                    self._candidate_color = None
                    self._candidate_started_at = None
                return VelocityCommand()
            self._last_seen_at = now

            angular = clamp(
                -self.config.align_kp * detection.x_error,
                -self.config.max_align_speed,
                self.config.max_align_speed,
            )
            if abs(detection.x_error) > self.config.align_tolerance:
                self.state = SortState.ALIGN
                return VelocityCommand(angular_z=angular)

            self.state = SortState.APPROACH
            contact_reached = (
                detection.range_m is not None
                and detection.range_m <= self.config.contact_distance
            ) or detection.area_ratio >= self.config.contact_area_ratio
            if not contact_reached:
                approach_speed = self.config.approach_speed
                if detection.range_m is not None:
                    range_error = max(
                        0.0,
                        detection.range_m - self.config.contact_distance,
                    )
                    approach_speed = clamp(
                        self.config.approach_range_kp * range_error,
                        self.config.minimum_approach_speed,
                        self.config.approach_speed,
                    )
                return VelocityCommand(
                    linear_x=approach_speed,
                    angular_z=angular,
                )
            if pose is None or self._origin is None:
                return self.stop_with_fault("odometry unavailable at contact")
            self._push_heading = pose.yaw
            self._push_started_at = now
            self._push_target_last_seen_at = now
            self._contact_area_ratio = detection.area_ratio
            self.state = SortState.PUSH
            return VelocityCommand()

        if self.state == SortState.PUSH:
            if pose is None or self._origin is None or color is None:
                return self.stop_with_fault("push state data unavailable")
            if (
                self._push_started_at is None
                or now - self._push_started_at
                >= self.config.maximum_push_duration
            ):
                return self.stop_with_fault("push timeout")
            push_detection = self._largest_detection(detections, color)
            if push_detection is not None:
                self._push_target_last_seen_at = now
            elif (
                self.config.push_target_lost_timeout > 0.0
                and self._push_target_last_seen_at is not None
                and now - self._push_target_last_seen_at
                >= self.config.push_target_lost_timeout
            ):
                return self.stop_with_fault("target lost while pushing")
            target_x, target_y = self._zone_in_odom(color)
            error_x = target_x - pose.x
            error_y = target_y - pose.y
            distance = math.hypot(error_x, error_y)
            if distance <= self.config.zone_tolerance:
                self.state = SortState.RELEASE
                self._release_started_at = now
                self._delivery_candidate_started_at = None
                self._push_started_at = None
                self._push_target_last_seen_at = None
                return VelocityCommand()

            cos_yaw = math.cos(pose.yaw)
            sin_yaw = math.sin(pose.yaw)
            robot_x = cos_yaw * error_x + sin_yaw * error_y
            robot_y = -sin_yaw * error_x + cos_yaw * error_y
            requested_speed = min(
                self.config.push_speed,
                self.config.push_kp * distance,
            )
            scale = requested_speed / distance
            yaw_error = normalize_angle(self._push_heading - pose.yaw)
            return VelocityCommand(
                linear_x=robot_x * scale,
                linear_y=robot_y * scale,
                angular_z=clamp(
                    self.config.yaw_kp * yaw_error,
                    -self.config.max_yaw_speed,
                    self.config.max_yaw_speed,
                ),
            )

        if self.state == SortState.RELEASE:
            if self._release_started_at is None:
                return self.stop_with_fault("release timer unavailable")
            if now - self._release_started_at < self.config.release_duration:
                return VelocityCommand(linear_x=-self.config.release_speed)
            if self.config.delivery_confirmation_timeout <= 0.0:
                return self._finish_release(now, color)

            delivery_detection = self._largest_detection(detections, color)
            maximum_delivery_area = self.config.delivery_maximum_area_ratio
            if self._contact_area_ratio is not None:
                maximum_delivery_area = min(
                    maximum_delivery_area,
                    self._contact_area_ratio
                    * self.config.delivery_maximum_area_scale,
                )
            delivery_visible = (
                delivery_detection is not None
                and abs(delivery_detection.x_error)
                <= self.config.delivery_maximum_x_error
                and delivery_detection.area_ratio
                <= maximum_delivery_area
            )
            if delivery_visible:
                if self._delivery_candidate_started_at is None:
                    self._delivery_candidate_started_at = now
                elif (
                    now - self._delivery_candidate_started_at
                    >= self.config.delivery_confirmation_duration
                ):
                    return self._finish_release(now, color)
            else:
                self._delivery_candidate_started_at = None
            if (
                now - self._release_started_at
                >= self.config.release_duration
                + self.config.delivery_confirmation_timeout
            ):
                return self.stop_with_fault("delivery not confirmed")
            return VelocityCommand()

        if self.state == SortState.RETURN:
            if pose is None or self._origin is None:
                return self.stop_with_fault("return state data unavailable")
            if (
                self._return_started_at is None
                or now - self._return_started_at
                >= self.config.maximum_return_duration
            ):
                return self.stop_with_fault("return timeout")

            if self.external_return_enabled:
                return VelocityCommand()

            while self._return_waypoint_index < len(self._return_waypoints):
                target_x, target_y = self._return_waypoints[
                    self._return_waypoint_index
                ]
                error_x = target_x - pose.x
                error_y = target_y - pose.y
                distance = math.hypot(error_x, error_y)
                if distance > self.config.return_tolerance:
                    break
                self._return_waypoint_index += 1
            else:
                self._complete_return(now)
                return VelocityCommand()

            cos_yaw = math.cos(pose.yaw)
            sin_yaw = math.sin(pose.yaw)
            robot_x = cos_yaw * error_x + sin_yaw * error_y
            robot_y = -sin_yaw * error_x + cos_yaw * error_y
            requested_speed = min(
                self.config.return_speed,
                self.config.return_kp * distance,
            )
            scale = requested_speed / distance
            yaw_error = normalize_angle(self._origin.yaw - pose.yaw)
            return VelocityCommand(
                linear_x=robot_x * scale,
                linear_y=robot_y * scale,
                angular_z=clamp(
                    self.config.yaw_kp * yaw_error,
                    -self.config.max_yaw_speed,
                    self.config.max_yaw_speed,
                ),
            )

        return self.stop_with_fault("invalid controller state")

    def _complete_return(self, now: float) -> None:
        if any(
            self.remaining.get(item, 0) != 0
            for item in self.color_order
        ):
            self.state = SortState.SEARCH
            self._search_started_at = now
        else:
            self.state = SortState.COMPLETE
            self._search_started_at = None
        self._no_target_started_at = None
        self._candidate_color = None
        self._candidate_started_at = None
        self._return_started_at = None
        self._return_waypoints = ()
        self._return_waypoint_index = 0

    @staticmethod
    def _largest_detection(
        detections: Iterable[Detection],
        color: str | None,
    ) -> Detection | None:
        matching = [item for item in detections if item.color == color]
        return max(matching, key=lambda item: item.area_ratio, default=None)

    def _largest_available_detection(
        self,
        detections: Iterable[Detection],
    ) -> Detection | None:
        available = [
            item
            for item in detections
            if item.color in self.color_order
            and self.remaining.get(item.color, 0) != 0
        ]
        return max(available, key=lambda item: item.area_ratio, default=None)

    def _select_next_color(self, advance: bool = False) -> None:
        if not any(self.remaining.get(color, 0) != 0 for color in self.color_order):
            self.state = SortState.COMPLETE
            return
        first_offset = 1 if advance else 0
        for step in range(len(self.color_order)):
            offset = first_offset + step
            index = (self._color_index + offset) % len(self.color_order)
            if self.remaining.get(self.color_order[index], 0) != 0:
                self._color_index = index
                return

    def _finish_release(
        self,
        now: float,
        color: str | None,
    ) -> VelocityCommand:
        if color is not None:
            self.processed[color] = self.processed.get(color, 0) + 1
            if self.remaining.get(color, 0) > 0:
                self.remaining[color] -= 1
        self.state = SortState.RETURN
        self._release_started_at = None
        self._delivery_candidate_started_at = None
        self._contact_area_ratio = None
        self._return_started_at = now
        self._return_waypoints = self._build_return_waypoints(color)
        self._return_waypoint_index = 0
        if any(
            self.remaining.get(item, 0) != 0
            for item in self.color_order
        ):
            self._select_next_color(advance=True)
        return VelocityCommand()

    def _zone_in_odom(self, color: str) -> tuple[float, float]:
        zone = self.zones[color]
        if not self.zones_relative_to_origin:
            return zone.x, zone.y
        return self._relative_point_in_odom(zone.x, zone.y)

    def _build_return_waypoints(
        self,
        delivered_color: str | None,
    ) -> tuple[tuple[float, float], ...]:
        if self._origin is None or delivered_color is None:
            return ()
        zone = self.zones[delivered_color]
        if self.zones_relative_to_origin:
            zone_x = zone.x
            zone_y = zone.y
        else:
            zone_x, zone_y = self._point_in_origin_frame(zone.x, zone.y)
        side = -1.0 if zone_y < 0.0 else 1.0
        corridor_y = side * max(
            self.config.return_corridor_offset,
            abs(zone_y) + self.config.return_corridor_margin,
        )
        return (
            self._relative_point_in_odom(zone_x, corridor_y),
            self._relative_point_in_odom(0.0, corridor_y),
            self._relative_point_in_odom(0.0, 0.0),
        )

    def _point_in_origin_frame(
        self,
        x: float,
        y: float,
    ) -> tuple[float, float]:
        if self._origin is None:
            raise RuntimeError("origin is not available")
        error_x = x - self._origin.x
        error_y = y - self._origin.y
        cos_yaw = math.cos(self._origin.yaw)
        sin_yaw = math.sin(self._origin.yaw)
        return (
            cos_yaw * error_x + sin_yaw * error_y,
            -sin_yaw * error_x + cos_yaw * error_y,
        )

    def _relative_point_in_odom(
        self,
        relative_x: float,
        relative_y: float,
    ) -> tuple[float, float]:
        if self._origin is None:
            raise RuntimeError("origin is not available")
        cos_yaw = math.cos(self._origin.yaw)
        sin_yaw = math.sin(self._origin.yaw)
        return (
            self._origin.x + cos_yaw * relative_x - sin_yaw * relative_y,
            self._origin.y + sin_yaw * relative_x + cos_yaw * relative_y,
        )
