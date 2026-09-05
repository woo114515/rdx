"""Hardware-independent motion request watchdog and limiter."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .behavior import VelocityCommand, clamp


def output_topic_has_conflict(
    publisher_count: int,
    own_publisher_count: int = 1,
) -> bool:
    """Return whether another node is publishing to the base command topic."""
    if publisher_count < 0 or own_publisher_count < 0:
        raise ValueError("publisher counts must be non-negative")
    return publisher_count > own_publisher_count


def navigation_controls_output(enabled: bool, sorting_phase: str) -> bool:
    """Select Nav2 requests only while the sorter is returning."""
    return enabled and sorting_phase == "return"


@dataclass(frozen=True)
class ObstacleConfig:
    stop_distance: float = 0.30
    movement_half_angle: float = math.radians(35.0)
    target_corridor_half_angle: float = math.radians(15.0)


def command_has_obstacle(
    command: VelocityCommand,
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    config: ObstacleConfig | None = None,
    ignore_forward_target_corridor: bool = False,
) -> bool:
    """Return whether valid scan points block the requested movement."""
    settings = config or ObstacleConfig()
    if settings.stop_distance <= 0.0:
        raise ValueError("stop_distance must be positive")
    if angle_increment == 0.0:
        raise ValueError("angle_increment must not be zero")

    translating = math.hypot(command.linear_x, command.linear_y) > 1e-6
    rotating = abs(command.angular_z) > 1e-6
    if not translating and not rotating:
        return False
    movement_angle = math.atan2(command.linear_y, command.linear_x)

    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < max(0.0, range_min) or distance > range_max:
            continue
        if distance >= settings.stop_distance:
            continue

        angle = angle_min + index * angle_increment
        if (
            ignore_forward_target_corridor
            and _angle_distance(angle, 0.0)
            <= settings.target_corridor_half_angle
        ):
            continue
        if rotating:
            return True
        if (
            translating
            and _angle_distance(angle, movement_angle)
            <= settings.movement_half_angle
        ):
            return True
    return False


def _angle_distance(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


@dataclass(frozen=True)
class SafetyConfig:
    request_timeout: float = 0.3
    maximum_linear_speed: float = 0.12
    maximum_angular_speed: float = 0.25


class CommandWatchdog:
    """Return bounded commands and stop when requests become stale."""

    def __init__(self, config: SafetyConfig | None = None) -> None:
        self.config = config or SafetyConfig()
        if self.config.request_timeout <= 0.0:
            raise ValueError("request_timeout must be positive")
        if self.config.maximum_linear_speed <= 0.0:
            raise ValueError("maximum_linear_speed must be positive")
        if self.config.maximum_angular_speed <= 0.0:
            raise ValueError("maximum_angular_speed must be positive")
        self._command = VelocityCommand()
        self._received_at: float | None = None
        self._emergency_stopped = False

    @property
    def emergency_stopped(self) -> bool:
        return self._emergency_stopped

    def update(self, now: float, command: VelocityCommand) -> None:
        self._command = command
        self._received_at = now

    def emergency_stop(self) -> None:
        self._emergency_stopped = True

    def output(self, now: float) -> VelocityCommand:
        if self._emergency_stopped or self._received_at is None:
            return VelocityCommand()
        if now - self._received_at > self.config.request_timeout:
            return VelocityCommand()

        linear_x = self._command.linear_x
        linear_y = self._command.linear_y
        magnitude = math.hypot(linear_x, linear_y)
        if magnitude > self.config.maximum_linear_speed:
            scale = self.config.maximum_linear_speed / magnitude
            linear_x *= scale
            linear_y *= scale
        return VelocityCommand(
            linear_x=linear_x,
            linear_y=linear_y,
            angular_z=clamp(
                self._command.angular_z,
                -self.config.maximum_angular_speed,
                self.config.maximum_angular_speed,
            ),
        )
