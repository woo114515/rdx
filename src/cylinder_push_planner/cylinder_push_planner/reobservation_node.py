"""Active, bounded viewpoint recovery for an incomplete cylinder snapshot."""

from __future__ import annotations

import json
import math
import signal
import threading
import time
from typing import Callable

import rclpy
from action_msgs.msg import GoalStatus
from color_object_sorter_interfaces.msg import (
    CylinderSnapshot,
    ValidatedCylinderArray,
)
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .execution_safety import nav2_path_result_error_code
from .geometry import (
    Target,
    Viewpoint,
    generate_reobservation_viewpoints,
    path_length,
    path_stays_outside,
    point_sets_stable,
    points_near_reference,
)
from .safety import navigation_safety_fault


class ReobservationNode(Node):
    """Move around an incomplete field and restart stationary collection."""

    def __init__(self) -> None:
        super().__init__("cylinder_reobservation")
        defaults = {
            "snapshot_topic": "/cylinder_snapshot/candidates",
            "validated_topic": "/cylinder_snapshot/validated_objects",
            "scan_topic": "/scan",
            "fixed_frame": "map",
            "robot_frame": "base_footprint",
            "compute_path_action": "/compute_path_to_pose",
            "follow_path_action": "/follow_path",
            "controller_server": "/controller_server",
            "controller_plugin": "FollowPath",
            "stop_topic": "/cmd_vel",
            "execution_enabled": False,
            "require_zero_lateral_velocity": True,
            "collection_duration": 5.0,
            "collection_timeout": 12.0,
            "stable_snapshot_updates": 8,
            "stable_candidate_motion": 0.06,
            "maximum_reference_distance": 0.50,
            "settling_duration": 1.0,
            "robot_center_clearance": 0.25,
            "occluded_object_allowance": 0.30,
            "viewpoint_travel_distance": 0.30,
            "minimum_viewpoint_angle": 0.10,
            "maximum_viewpoint_angle": math.radians(25.0),
            "maximum_single_path_length": 0.75,
            "maximum_total_path_length": 1.0,
            "maximum_attempts": 3,
            "right_preference_bonus": 0.05,
            "input_timeout": 1.0,
            "scan_timeout": 1.0,
            "navigation_timeout": 20.0,
            "transform_timeout": 0.20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, "/cylinder_reobservation/status", latched
        )
        self._path_publisher = self.create_publisher(
            Path, "/cylinder_reobservation/path", latched
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, "/cylinder_reobservation/markers", latched
        )
        self._stop_publisher = self.create_publisher(
            Twist, self._string("stop_topic"), 10
        )
        self.create_subscription(
            CylinderSnapshot,
            self._string("snapshot_topic"),
            self._on_snapshot,
            10,
        )
        self.create_subscription(
            ValidatedCylinderArray,
            self._string("validated_topic"),
            self._on_validated,
            latched,
        )
        self.create_subscription(
            LaserScan,
            self._string("scan_topic"),
            self._on_scan,
            qos_profile_sensor_data,
        )

        self._compute_path = ActionClient(
            self,
            ComputePathToPose,
            self._string("compute_path_action"),
        )
        self._follow_path = ActionClient(
            self,
            FollowPath,
            self._string("follow_path_action"),
        )
        parameter_service = (
            self._string("controller_server").rstrip("/") + "/get_parameters"
        )
        self._controller_parameters = self.create_client(
            GetParameters, parameter_service
        )
        self._validation_reset = self.create_client(
            Trigger, "/cylinder_validation/reset"
        )
        self._snapshot_reset = self.create_client(
            Trigger, "/cylinder_snapshot/reset"
        )
        self._snapshot_start = self.create_client(
            Trigger, "/cylinder_snapshot/start"
        )
        self._snapshot_lock = self.create_client(
            Trigger, "/cylinder_snapshot/lock"
        )
        self.create_service(Trigger, "/cylinder_reobservation/start", self._start)
        self.create_service(
            Trigger, "/cylinder_reobservation/cancel", self._cancel
        )
        self.create_service(Trigger, "/cylinder_reobservation/reset", self._reset)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._state = "idle"
        self._generation = 0
        self._attempts = 0
        self._total_path_length = 0.0
        self._collection_started = 0.0
        self._settle_until = 0.0
        self._navigation_started = 0.0
        self._latest_snapshot: CylinderSnapshot | None = None
        self._snapshot_received = 0.0
        self._previous_snapshot_points: tuple[tuple[float, float], ...] = ()
        self._stable_snapshot_updates = 0
        self._reference_points: tuple[tuple[float, float], ...] = ()
        self._snapshot_gate_reason = "waiting for candidate snapshot"
        self._latest_validated: ValidatedCylinderArray | None = None
        self._last_scan_received = 0.0
        self._viewpoints: list[Viewpoint] = []
        self._plan_index = 0
        self._path_options: list[tuple[Viewpoint, Path, float]] = []
        self._selected: tuple[Viewpoint, Path, float] | None = None
        self._nav_goal_handle = None
        self._stop_burst_remaining = 0
        self._tf_failure_count = 0
        self.create_timer(0.1, self._tick)
        self._publish_status("idle", "call /cylinder_reobservation/start")
        self.get_logger().info(
            "Reobservation ready; navigation execution is explicitly gated"
        )

    def _start(self, request, response):
        del request
        if self._state not in {"idle", "complete", "failed", "cancelled", "preview"}:
            response.success = False
            response.message = f"reobservation is already {self._state}"
            return response
        missing = [
            name
            for name, client in (
                ("validation reset", self._validation_reset),
                ("snapshot reset", self._snapshot_reset),
                ("snapshot start", self._snapshot_start),
            )
            if not client.service_is_ready()
        ]
        if missing:
            response.success = False
            response.message = "services unavailable: " + ", ".join(missing)
            return response
        if not self._compute_path.wait_for_server(timeout_sec=0.0):
            response.success = False
            response.message = "ComputePathToPose action server is unavailable"
            return response
        if self._bool("execution_enabled") and not self._follow_path.wait_for_server(
            timeout_sec=0.0
        ):
            response.success = False
            response.message = "FollowPath action server is unavailable"
            return response
        if (
            self._bool("execution_enabled")
            and self._bool("require_zero_lateral_velocity")
            and not self._controller_parameters.service_is_ready()
        ):
            response.success = False
            response.message = "controller parameter services are unavailable"
            return response

        self._generation += 1
        self._attempts = 0
        self._total_path_length = 0.0
        self._reference_points = ()
        self._selected = None
        if self._bool("execution_enabled") and self._bool(
            "require_zero_lateral_velocity"
        ):
            self._begin_controller_preflight(self._generation)
        else:
            self._begin_new_collection(self._generation)
        response.success = True
        mode = "execute" if self._bool("execution_enabled") else "preview"
        response.message = f"reobservation started in {mode} mode"
        return response

    def _begin_controller_preflight(self, generation: int) -> None:
        self._state = "preflight"
        plugin = self._string("controller_plugin")
        names = [f"{plugin}.min_vel_y", f"{plugin}.max_vel_y"]
        self._publish_status("preflight", "checking Nav2 lateral velocity limits")
        request = GetParameters.Request()
        request.names = names
        future = self._controller_parameters.call_async(request)

        def done(completed) -> None:
            if generation != self._generation:
                return
            try:
                values = completed.result().values
                lateral_limits = [value.double_value for value in values]
            except Exception as error:
                self._abort(f"controller parameter check failed: {error}")
                return
            if len(lateral_limits) != 2 or any(
                abs(value) > 1e-6 for value in lateral_limits
            ):
                self._abort(
                    "Nav2 lateral velocity must be disabled before reobservation"
                )
                return
            self._begin_new_collection(generation)

        future.add_done_callback(done)

    def _cancel(self, request, response):
        del request
        if self._state in {"idle", "complete", "failed", "cancelled"}:
            response.success = False
            response.message = "no active reobservation"
            return response
        self._terminate("cancelled", "cancel requested")
        response.success = True
        response.message = "reobservation cancelled; zero velocity requested"
        return response

    def _reset(self, request, response):
        del request
        self._terminate("idle", "reset")
        self._latest_snapshot = None
        self._latest_validated = None
        self._previous_snapshot_points = ()
        self._stable_snapshot_updates = 0
        self._reference_points = ()
        self._attempts = 0
        self._total_path_length = 0.0
        self._clear_visualization()
        if self._validation_reset.service_is_ready():
            self._validation_reset.call_async(Trigger.Request())
        if self._snapshot_reset.service_is_ready():
            self._snapshot_reset.call_async(Trigger.Request())
        response.success = True
        response.message = "reobservation reset; zero velocity requested"
        return response

    def _on_snapshot(self, message: CylinderSnapshot) -> None:
        if self._state == "collecting":
            self._latest_snapshot = message
            self._snapshot_received = self._now()
            points = tuple(
                (item.position.x, item.position.y) for item in message.candidates
            )
            if self._reference_points and not points_near_reference(
                points,
                self._reference_points,
                self._float("maximum_reference_distance"),
            ):
                self._stable_snapshot_updates = 0
                self._snapshot_gate_reason = "candidate outside the accepted field"
            elif point_sets_stable(
                self._previous_snapshot_points,
                points,
                self._float("stable_candidate_motion"),
            ):
                self._stable_snapshot_updates += 1
                self._snapshot_gate_reason = "candidate snapshot is stabilizing"
            else:
                self._stable_snapshot_updates = 1 if points else 0
                self._snapshot_gate_reason = "candidate set changed"
            self._previous_snapshot_points = points

    def _on_validated(self, message: ValidatedCylinderArray) -> None:
        if self._state != "collecting":
            return
        self._latest_validated = message
        if message.ready and message.locked:
            self._state = "complete"
            self._publish_zero()
            if self._snapshot_lock.service_is_ready():
                self._snapshot_lock.call_async(Trigger.Request())
            self._publish_status(
                "complete", "complete validated inventory locked"
            )

    def _on_scan(self, message: LaserScan) -> None:
        del message
        self._last_scan_received = self._now()

    def _tick(self) -> None:
        if self._stop_burst_remaining > 0:
            self._publish_zero()
            self._stop_burst_remaining -= 1

        now = self._now()
        if self._state == "collecting":
            if now - self._collection_started < self._float("collection_duration"):
                return
            if self._latest_validated is not None:
                if self._latest_validated.ready and self._latest_validated.locked:
                    return
            if self._latest_snapshot is None:
                self._abort("no cylinder snapshot received")
                return
            if now - self._snapshot_received > self._float("input_timeout"):
                self._abort("cylinder snapshot is stale")
                return
            required_updates = self._int("stable_snapshot_updates")
            if self._stable_snapshot_updates < required_updates:
                if now - self._collection_started >= self._float("collection_timeout"):
                    self._abort(
                        "candidate snapshot did not stabilize: "
                        + self._snapshot_gate_reason
                    )
                return
            if self._attempts >= self._int("maximum_attempts"):
                self._abort("maximum viewpoint attempts reached")
                return
            self._begin_path_planning()
        elif self._state == "navigating":
            fault = navigation_safety_fault(
                now,
                self._last_scan_received,
                self._navigation_started,
                self._float("scan_timeout"),
                self._float("navigation_timeout"),
            )
            if fault is not None:
                self._abort(fault)
                return
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._string("fixed_frame"),
                    self._string("robot_frame"),
                    Time(),
                    timeout=Duration(seconds=0.02),
                )
                self._tf_failure_count = 0
                if self._selected is None:
                    self._abort("selected viewpoint was lost during navigation")
                    return
                viewpoint = self._selected[0]
                position = transform.transform.translation
                if math.hypot(
                    position.x - viewpoint.envelope_x,
                    position.y - viewpoint.envelope_y,
                ) < viewpoint.minimum_center_radius:
                    self._abort("robot entered the cylinder clearance envelope")
                    return
            except TransformException:
                self._tf_failure_count += 1
                if self._tf_failure_count >= 3:
                    self._abort("robot transform lost during navigation")
        elif self._state == "settling" and now >= self._settle_until:
            self._begin_new_collection(self._generation)

    def _begin_new_collection(self, generation: int) -> None:
        self._state = "restarting"
        self._latest_snapshot = None
        self._latest_validated = None
        self._previous_snapshot_points = ()
        self._stable_snapshot_updates = 0
        self._snapshot_gate_reason = "waiting for candidate snapshot"
        self._publish_status("restarting", "clearing previous evidence")
        self._call_trigger(
            self._validation_reset,
            "validation reset",
            generation,
            lambda: self._call_trigger(
                self._snapshot_reset,
                "snapshot reset",
                generation,
                lambda: self._call_trigger(
                    self._snapshot_start,
                    "snapshot start",
                    generation,
                    lambda: self._collection_ready(generation),
                ),
            ),
        )

    def _collection_ready(self, generation: int) -> None:
        if generation != self._generation:
            return
        self._state = "collecting"
        self._collection_started = self._now()
        self._publish_status("collecting", "stationary snapshot collection")

    def _call_trigger(
        self,
        client,
        label: str,
        generation: int,
        on_success: Callable[[], None],
    ) -> None:
        future = client.call_async(Trigger.Request())

        def done(completed) -> None:
            if generation != self._generation:
                return
            try:
                result = completed.result()
            except Exception as error:
                self._abort(f"{label} failed: {error}")
                return
            if not result.success:
                self._abort(f"{label} rejected: {result.message}")
                return
            on_success()

        future.add_done_callback(done)

    def _begin_path_planning(self) -> None:
        assert self._latest_snapshot is not None
        targets = tuple(
            Target(
                candidate_id=item.candidate_id,
                x=item.position.x,
                y=item.position.y,
                color="",
                radius=item.radius,
            )
            for item in self._latest_snapshot.candidates
        )
        if not targets:
            self._abort("no spatial candidates available for viewpoint planning")
            return
        if not self._reference_points:
            self._reference_points = tuple((item.x, item.y) for item in targets)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._string("fixed_frame"),
                self._string("robot_frame"),
                Time(),
                timeout=Duration(seconds=self._float("transform_timeout")),
            )
            translation = transform.transform.translation
            self._viewpoints = list(
                generate_reobservation_viewpoints(
                    targets,
                    translation.x,
                    translation.y,
                    self._float("robot_center_clearance")
                    + self._float("occluded_object_allowance"),
                    self._float("viewpoint_travel_distance"),
                    self._float("minimum_viewpoint_angle"),
                    self._float("maximum_viewpoint_angle"),
                )
            )
        except (TransformException, ValueError) as error:
            self._abort(f"cannot generate viewpoints: {error}")
            return

        self._state = "planning"
        self._plan_index = 0
        self._path_options = []
        self._publish_viewpoint_markers(self._viewpoints, None)
        self._publish_status("planning", self._latest_snapshot.status)
        self._request_next_path(self._generation)

    def _request_next_path(self, generation: int) -> None:
        if generation != self._generation:
            return
        if self._plan_index >= len(self._viewpoints):
            self._select_path(generation)
            return
        viewpoint = self._viewpoints[self._plan_index]
        self._plan_index += 1
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(viewpoint)
        goal.planner_id = ""
        goal.use_start = False
        future = self._compute_path.send_goal_async(goal)

        def response(completed) -> None:
            try:
                handle = completed.result()
            except Exception:
                self._request_next_path(generation)
                return
            if generation != self._generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._request_next_path(generation)
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._path_result(
                    result, viewpoint, generation
                )
            )

        future.add_done_callback(response)

    def _path_result(self, future, viewpoint: Viewpoint, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            wrapped = future.result()
            result = wrapped.result
            path = result.path
        except Exception:
            self._request_next_path(generation)
            return
        points = tuple((pose.pose.position.x, pose.pose.position.y) for pose in path.poses)
        length = path_length(points)
        valid = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and nav2_path_result_error_code(result) == 0
            and len(points) >= 2
            and length <= self._float("maximum_single_path_length")
            and self._total_path_length + length
            <= self._float("maximum_total_path_length")
            and path_stays_outside(
                points,
                viewpoint.envelope_x,
                viewpoint.envelope_y,
                viewpoint.minimum_center_radius,
            )
        )
        if valid:
            self._path_options.append((viewpoint, path, length))
        self._request_next_path(generation)

    def _select_path(self, generation: int) -> None:
        if generation != self._generation:
            return
        if not self._path_options:
            self._abort("Nav2 found no short path outside the cylinder envelope")
            return
        bonus = self._float("right_preference_bonus")
        self._selected = min(
            self._path_options,
            key=lambda option: option[2] - (bonus if option[0].side == "right" else 0.0),
        )
        viewpoint, path, length = self._selected
        self._path_publisher.publish(path)
        self._publish_viewpoint_markers(self._viewpoints, viewpoint)
        if not self._bool("execution_enabled"):
            self._state = "preview"
            self._publish_zero()
            self._publish_status(
                "preview",
                f"selected {viewpoint.side} path length={length:.3f}; execution disabled",
            )
            return

        self._state = "preparing_navigation"
        self._publish_status(
            "preparing_navigation", "clearing snapshot before robot movement"
        )
        self._call_trigger(
            self._validation_reset,
            "validation reset",
            generation,
            lambda: self._call_trigger(
                self._snapshot_reset,
                "snapshot reset",
                generation,
                lambda: self._send_navigation_goal(generation),
            ),
        )

    def _send_navigation_goal(self, generation: int) -> None:
        if generation != self._generation or self._selected is None:
            return
        viewpoint, checked_path, length = self._selected
        goal = FollowPath.Goal()
        goal.path = checked_path
        goal.controller_id = ""
        goal.goal_checker_id = ""
        future = self._follow_path.send_goal_async(goal)

        def response(completed) -> None:
            try:
                handle = completed.result()
            except Exception as error:
                self._abort(f"FollowPath request failed: {error}")
                return
            if generation != self._generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._abort("FollowPath goal rejected")
                return
            self._nav_goal_handle = handle
            self._attempts += 1
            self._total_path_length += length
            self._navigation_started = self._now()
            self._tf_failure_count = 0
            self._state = "navigating"
            self._publish_status(
                "navigating",
                f"attempt={self._attempts};side={viewpoint.side};"
                f"checked_path_length={length:.3f}",
            )
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._navigation_result(result, generation)
            )

        future.add_done_callback(response)

    def _navigation_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        self._nav_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self._abort(f"FollowPath result failed: {error}")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"FollowPath status {wrapped.status}")
            return
        self._publish_zero()
        self._state = "settling"
        self._settle_until = self._now() + self._float("settling_duration")
        self._publish_status("settling", "waiting for TF and scan to stabilize")

    def _pose(self, viewpoint: Viewpoint) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._string("fixed_frame")
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = viewpoint.x
        pose.pose.position.y = viewpoint.y
        pose.pose.orientation.z = math.sin(viewpoint.yaw / 2.0)
        pose.pose.orientation.w = math.cos(viewpoint.yaw / 2.0)
        return pose

    def _terminate(self, state: str, reason: str) -> None:
        self._generation += 1
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._state = state
        self._stop_burst_remaining = 5
        self._publish_zero()
        self._publish_status(state, reason)

    def _abort(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._terminate("failed", reason)

    def _publish_zero(self) -> None:
        self._stop_publisher.publish(Twist())

    def _publish_status(self, state: str, reason: str) -> None:
        payload = {
            "attempt": self._attempts,
            "execution_enabled": self._bool("execution_enabled"),
            "reason": reason,
            "state": state,
            "total_path_length": round(self._total_path_length, 3),
        }
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_viewpoint_markers(
        self, viewpoints: list[Viewpoint], selected: Viewpoint | None
    ) -> None:
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        if not viewpoints:
            self._marker_publisher.publish(output)
            return
        reference = viewpoints[0]
        envelope = Marker()
        envelope.header.frame_id = self._string("fixed_frame")
        envelope.header.stamp = self.get_clock().now().to_msg()
        envelope.ns = "reobservation_clearance"
        envelope.id = 1
        envelope.type = Marker.LINE_STRIP
        envelope.action = Marker.ADD
        envelope.scale.x = 0.02
        envelope.color.r = 1.0
        envelope.color.g = 0.6
        envelope.color.a = 0.9
        for index in range(65):
            angle = 2.0 * math.pi * index / 64.0
            envelope.points.append(
                Point(
                    x=reference.envelope_x
                    + reference.minimum_center_radius * math.cos(angle),
                    y=reference.envelope_y
                    + reference.minimum_center_radius * math.sin(angle),
                )
            )
        output.markers.append(envelope)
        for index, viewpoint in enumerate(viewpoints, start=2):
            marker = Marker()
            marker.header = envelope.header
            marker.ns = "reobservation_viewpoints"
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self._pose(viewpoint).pose
            marker.scale.x = 0.25
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            marker.color.g = 1.0
            marker.color.b = 1.0 if viewpoint.side == "left" else 0.0
            marker.color.r = 1.0 if selected == viewpoint else 0.0
            marker.color.a = 1.0
            output.markers.append(marker)
        self._marker_publisher.publish(output)

    def _clear_visualization(self) -> None:
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._marker_publisher.publish(MarkerArray(markers=[clear]))
        self._path_publisher.publish(Path())

    def prepare_shutdown(self) -> None:
        """Cancel navigation and publish zero while the ROS context is valid."""

        self._generation += 1
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._state = "cancelled"
        for _unused in range(5):
            self._publish_zero()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def destroy_node(self):
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ReobservationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    shutdown_requested = threading.Event()

    def request_shutdown(signum, frame) -> None:
        del signum, frame
        shutdown_requested.set()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        executor.add_node(node)
        while rclpy.ok() and not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.prepare_shutdown()
            deadline = time.monotonic() + 0.2
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
