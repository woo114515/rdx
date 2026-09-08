"""Explicitly armed execution of one approach, push, return and rescan cycle."""

from __future__ import annotations

import json
import math
import signal
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from color_object_sorter_interfaces.msg import ValidatedCylinderArray
from color_object_sorter_interfaces.srv import (
    SetObjectInventory,
    SetSnapshotExclusions,
)
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rcl_interfaces.srv import GetParameters
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
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .execution_safety import (
    approach_watchdog_fault,
    front_target_present,
    nav2_path_result_error_code,
    obstacle_behind,
    path_stays_outside_polygon,
    plan_inputs_fault,
    point_outside_polygon,
    retreat_motion,
    retreat_pose_step,
    tracking_command,
    unexpected_obstacle_ahead,
)
from .geometry import path_length
from .inventory_cycle import inventory_after_delivery


class ApproachExecutionNode(Node):
    """Run separately armed stages and command zero velocity on every fault."""

    def __init__(self) -> None:
        super().__init__("cylinder_push_approach_execution")
        defaults = {
            "execution_enabled": False,
            "plan_status_topic": "/cylinder_push_plan/status",
            "snapshot_topic": "/cylinder_snapshot/validated_objects",
            "staging_pose_topic": "/cylinder_push_plan/staging_pose",
            "approach_path_topic": "/cylinder_push_plan/approach_path",
            "target_path_topic": "/cylinder_push_plan/target_path",
            "push_path_topic": "/cylinder_push_plan/robot_push_path",
            "return_path_topic": "/cylinder_push_plan/return_path",
            "keepout_topic": "/cylinder_push_plan/remaining_keepout",
            "checked_path_topic": "/cylinder_push_execution/checked_approach_path",
            "scan_topic": "/scan",
            "stop_topic": "/cmd_vel",
            "fixed_frame": "map",
            "robot_frame": "base_footprint",
            "compute_path_action": "/compute_path_to_pose",
            "follow_path_action": "/follow_path",
            "controller_server": "/controller_server",
            "controller_plugin": "FollowPath",
            "goal_checker_id": "task3_goal_checker",
            "arming_timeout": 10.0,
            "plan_timeout": 120.0,
            "scan_timeout": 0.75,
            "navigation_timeout": 30.0,
            "transform_timeout": 0.20,
            "maximum_approach_length": 3.0,
            "maximum_goal_error": 0.15,
            "require_zero_lateral_velocity": True,
            "contact_speed": 0.03,
            "contact_maximum_angular_speed": 0.15,
            "contact_acquire_maximum_distance": 0.65,
            "contact_goal_tolerance": 0.04,
            "contact_timeout": 12.0,
            "push_speed": 0.06,
            "push_angular_gain": 1.5,
            "push_maximum_angular_speed": 0.25,
            "push_lookahead": 0.12,
            "push_goal_tolerance": 0.06,
            "maximum_push_deviation": 0.12,
            "push_timeout": 45.0,
            "target_maximum_distance": 0.32,
            "target_corridor_half_width": 0.07,
            "target_loss_timeout": 0.60,
            "obstacle_stop_distance": 0.32,
            "obstacle_half_width": 0.18,
            "release_distance": 0.15,
            "release_speed": 0.04,
            "release_timeout": 8.0,
            "maximum_release_lateral_deviation": 0.05,
            "maximum_release_heading_error": 0.15,
            "maximum_release_pose_step": 0.05,
            "rear_stop_distance": 0.25,
            "return_timeout": 45.0,
            "maximum_return_length": 3.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, "/cylinder_push_execution/status", latched
        )
        self._checked_path_publisher = self.create_publisher(
            Path, self._string("checked_path_topic"), latched
        )
        self._stop_publisher = self.create_publisher(
            Twist, self._string("stop_topic"), 10
        )
        self.create_subscription(
            String,
            self._string("plan_status_topic"),
            self._on_plan_status,
            latched,
        )
        self.create_subscription(
            ValidatedCylinderArray,
            self._string("snapshot_topic"),
            self._on_snapshot,
            latched,
        )
        self.create_subscription(
            PoseStamped,
            self._string("staging_pose_topic"),
            self._on_staging_pose,
            latched,
        )
        self.create_subscription(
            Path,
            self._string("approach_path_topic"),
            self._on_approach_path,
            latched,
        )
        self.create_subscription(
            Path,
            self._string("target_path_topic"),
            self._on_target_path,
            latched,
        )
        self.create_subscription(
            Path, self._string("push_path_topic"), self._on_push_path, latched
        )
        self.create_subscription(
            Path, self._string("return_path_topic"), self._on_return_path, latched
        )
        self.create_subscription(
            PolygonStamped,
            self._string("keepout_topic"),
            self._on_keepout,
            latched,
        )
        self.create_subscription(
            LaserScan,
            self._string("scan_topic"),
            self._on_scan,
            qos_profile_sensor_data,
        )

        self._compute_path = ActionClient(
            self, ComputePathToPose, self._string("compute_path_action")
        )
        self._follow_path = ActionClient(
            self, FollowPath, self._string("follow_path_action")
        )
        parameter_service = (
            self._string("controller_server").rstrip("/") + "/get_parameters"
        )
        self._controller_parameters = self.create_client(
            GetParameters, parameter_service
        )
        self.create_service(Trigger, "/cylinder_push_execution/arm", self._arm)
        self.create_service(
            Trigger, "/cylinder_push_execution/start_approach", self._start
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/cancel", self._cancel
        )
        self.create_service(Trigger, "/cylinder_push_execution/reset", self._reset)
        self.create_service(
            Trigger, "/cylinder_push_execution/reset_task", self._reset_task
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/arm_contact", self._arm_contact
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/start_contact", self._start_contact
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/arm_push", self._arm_push
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/start_push", self._start_push
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/arm_return", self._arm_return
        )
        self.create_service(
            Trigger, "/cylinder_push_execution/start_return", self._start_return
        )
        self.create_service(
            Trigger,
            "/cylinder_push_execution/finalize_delivery",
            self._finalize_delivery,
        )

        self._validation_reset = self.create_client(
            Trigger, "/cylinder_validation/reset"
        )
        self._plan_reset = self.create_client(Trigger, "/cylinder_push_plan/reset")
        self._snapshot_reset = self.create_client(
            Trigger, "/cylinder_snapshot/reset"
        )
        self._inventory_update = self.create_client(
            SetObjectInventory, "/cylinder_snapshot/set_inventory"
        )
        self._exclusion_update = self.create_client(
            SetSnapshotExclusions, "/cylinder_snapshot/set_exclusions"
        )
        self._snapshot_start = self.create_client(
            Trigger, "/cylinder_snapshot/start"
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._state = "idle"
        self._generation = 0
        self._plan_ready = False
        self._plan_received = 0.0
        self._plan_stamp: tuple[int, int] | None = None
        self._announced_plan_stamp: tuple[int, int] | None = None
        self._plan_target_id = -1
        self._plan_target_color = ""
        self._plan_destination: tuple[float, float] | None = None
        self._plan_exclusion_radius = 0.0
        self._delivered_destinations: list[tuple[float, float]] = []
        self._snapshot: ValidatedCylinderArray | None = None
        self._snapshot_ready = False
        self._snapshot_locked = False
        self._staging_pose: PoseStamped | None = None
        self._approach_preview: Path | None = None
        self._target_path: Path | None = None
        self._push_path: Path | None = None
        self._return_path: Path | None = None
        self._keepout: PolygonStamped | None = None
        self._last_scan_received = 0.0
        self._last_scan: LaserScan | None = None
        self._armed_until = 0.0
        self._navigation_started = 0.0
        self._nav_goal_handle = None
        self._compute_goal_handle = None
        self._stop_burst_remaining = 0
        self._tf_failure_count = 0
        self._push_progress = 0
        self._contact_origin: tuple[float, float] | None = None
        self._target_last_seen = 0.0
        self._release_origin: tuple[float, float, float] | None = None
        self._release_previous_pose: tuple[float, float, float] | None = None
        self._return_home: PoseStamped | None = None
        self.create_timer(0.1, self._tick)
        self._publish_status("idle", "generate a preview, then arm")
        self.get_logger().info(
            "Gated push-cycle execution ready; execution is disabled unless explicitly enabled"
        )

    def _on_plan_status(self, message: String) -> None:
        if not self._accept_plan_updates():
            return
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self._plan_ready = False
            return
        self._plan_ready = bool(payload.get("ready", False)) and not bool(
            payload.get("motion_output", True)
        )
        plan_stamp = payload.get("plan_stamp")
        if (
            isinstance(plan_stamp, list)
            and len(plan_stamp) == 2
            and all(isinstance(value, int) for value in plan_stamp)
        ):
            self._plan_stamp = (plan_stamp[0], plan_stamp[1])
        else:
            self._plan_stamp = None
            self._plan_ready = False
        self._plan_target_id = int(payload.get("target_id", -1))
        self._plan_target_color = str(payload.get("target_color", ""))
        destination = payload.get("destination")
        if (
            isinstance(destination, list)
            and len(destination) == 2
            and all(isinstance(value, (int, float)) for value in destination)
            and all(math.isfinite(float(value)) for value in destination)
        ):
            self._plan_destination = (
                float(destination[0]),
                float(destination[1]),
            )
        else:
            self._plan_destination = None
            self._plan_ready = False
        try:
            self._plan_exclusion_radius = float(
                payload.get("delivered_exclusion_radius", 0.0)
            )
        except (TypeError, ValueError):
            self._plan_exclusion_radius = 0.0
            self._plan_ready = False
        if not math.isfinite(self._plan_exclusion_radius) or (
            self._plan_exclusion_radius <= 0.0
        ):
            self._plan_ready = False
        self._maybe_announce_fresh_plan()

    def _on_snapshot(self, message: ValidatedCylinderArray) -> None:
        if not self._accept_plan_updates():
            return
        self._snapshot = message
        self._snapshot_ready = bool(message.ready)
        self._snapshot_locked = bool(message.locked)
        self._maybe_announce_fresh_plan()

    def _on_staging_pose(self, message: PoseStamped) -> None:
        if not self._accept_plan_updates():
            return
        self._staging_pose = message if message.header.frame_id else None
        self._maybe_announce_fresh_plan()

    def _on_approach_path(self, message: Path) -> None:
        if not self._accept_plan_updates():
            return
        self._approach_preview = message if len(message.poses) >= 2 else None
        self._maybe_announce_fresh_plan()

    def _on_target_path(self, message: Path) -> None:
        if not self._accept_plan_updates():
            return
        self._target_path = message if len(message.poses) >= 2 else None
        self._maybe_announce_fresh_plan()

    def _on_push_path(self, message: Path) -> None:
        if not self._accept_plan_updates():
            return
        self._push_path = message if len(message.poses) >= 2 else None
        self._maybe_announce_fresh_plan()

    def _on_return_path(self, message: Path) -> None:
        if not self._accept_plan_updates():
            return
        self._return_path = message if len(message.poses) >= 2 else None
        self._maybe_announce_fresh_plan()

    def _on_keepout(self, message: PolygonStamped) -> None:
        if not self._accept_plan_updates():
            return
        # An empty, stamped polygon is the valid final-cylinder case: there
        # are no remaining cylinders to protect. An unstamped message is the
        # planner's reset sentinel and means that no plan is available.
        self._keepout = message if message.header.frame_id else None
        self._maybe_announce_fresh_plan()

    @staticmethod
    def _message_stamp(message) -> tuple[int, int]:
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def _maybe_announce_fresh_plan(self) -> None:
        """Publish readiness only after every plan part has the new stamp."""

        if not self._plan_ready or self._plan_stamp is None:
            return
        if not self._snapshot_ready or not self._snapshot_locked:
            return
        messages = (
            self._staging_pose,
            self._approach_preview,
            self._target_path,
            self._push_path,
            self._return_path,
            self._keepout,
        )
        if any(message is None for message in messages):
            return
        if any(self._message_stamp(message) != self._plan_stamp for message in messages):
            return
        if self._announced_plan_stamp == self._plan_stamp:
            return
        self._announced_plan_stamp = self._plan_stamp
        self._plan_received = self._now()
        self._state = "idle"
        self._publish_status("idle", "fresh complete push preview received")

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan = message
        self._last_scan_received = self._now()

    def _arm(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._service_failure(response, "execution_enabled is false")
        if self._state not in {"idle", "failed", "cancelled"}:
            return self._service_failure(response, f"executor is {self._state}")
        fault = self._input_fault()
        if fault is not None:
            return self._service_failure(response, fault)
        if self._now() - self._plan_received > self._float("plan_timeout"):
            return self._service_failure(response, "push preview is stale")
        if self._now() - self._last_scan_received > self._float("scan_timeout"):
            return self._service_failure(response, "laser scan is missing or stale")
        if not self._compute_path.wait_for_server(timeout_sec=0.0):
            return self._service_failure(
                response, "ComputePathToPose action server is unavailable"
            )
        if not self._follow_path.wait_for_server(timeout_sec=0.0):
            return self._service_failure(
                response, "FollowPath action server is unavailable"
            )
        try:
            self._tf_buffer.lookup_transform(
                self._string("fixed_frame"),
                self._string("robot_frame"),
                Time(),
                timeout=Duration(seconds=self._float("transform_timeout")),
            )
        except TransformException as error:
            return self._service_failure(response, f"robot transform unavailable: {error}")
        if self._bool("require_zero_lateral_velocity"):
            if not self._controller_parameters.service_is_ready():
                return self._service_failure(
                    response, "controller parameter service is unavailable"
                )
            self._begin_controller_preflight()
            response.success = True
            response.message = "controller preflight started; wait for state=armed"
            return response
        self._finish_arming()
        response.success = True
        response.message = "approach armed"
        return response

    def _begin_controller_preflight(self) -> None:
        self._generation += 1
        generation = self._generation
        self._state = "preflight"
        self._publish_status("preflight", "checking Nav2 lateral velocity limits")
        plugin = self._string("controller_plugin")
        request = GetParameters.Request()
        request.names = [f"{plugin}.min_vel_y", f"{plugin}.max_vel_y"]
        future = self._controller_parameters.call_async(request)

        def done(completed) -> None:
            if generation != self._generation:
                return
            try:
                values = completed.result().values
                limits = [value.double_value for value in values]
            except Exception as error:
                self._abort(f"controller parameter check failed: {error}")
                return
            if len(limits) != 2 or any(abs(value) > 1e-6 for value in limits):
                self._abort("Nav2 lateral velocity must be disabled")
                return
            self._finish_arming()

        future.add_done_callback(done)

    def _finish_arming(self) -> None:
        self._state = "armed"
        self._armed_until = self._now() + self._float("arming_timeout")
        self._publish_status("armed", "call start_approach before arm expires")

    def _start(self, request, response):
        del request
        if self._state != "armed":
            return self._service_failure(response, f"executor is not armed: {self._state}")
        if self._now() > self._armed_until:
            self._terminate("cancelled", "arming window expired")
            return self._service_failure(response, "arming window expired")
        fault = self._input_fault()
        if fault is not None:
            self._abort(fault)
            return self._service_failure(response, fault)
        self._generation += 1
        generation = self._generation
        self._state = "checking_path"
        self._publish_status("checking_path", "requesting Nav2 collision-checked path")
        goal = ComputePathToPose.Goal()
        goal.goal = self._copy_staging_with_current_stamp()
        goal.planner_id = ""
        goal.use_start = False
        future = self._compute_path.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._compute_response(completed, generation)
        )
        response.success = True
        response.message = "approach path check started"
        return response

    def _compute_response(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._abort(f"ComputePathToPose request failed: {error}")
            return
        if not handle.accepted:
            self._abort("ComputePathToPose goal rejected")
            return
        self._compute_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._compute_result(completed, generation)
        )

    def _compute_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        self._compute_goal_handle = None
        try:
            wrapped = future.result()
            result = wrapped.result
            path = result.path
        except Exception as error:
            self._abort(f"ComputePathToPose result failed: {error}")
            return
        points = tuple(
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        )
        error_code = nav2_path_result_error_code(result)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or error_code != 0:
            self._abort(f"Nav2 path planning failed with code {error_code}")
            return
        if len(points) < 2:
            self._abort("Nav2 returned an empty approach path")
            return
        length = path_length(points)
        if length > self._float("maximum_approach_length"):
            self._abort(f"Nav2 approach is too long: {length:.3f} m")
            return
        assert self._keepout is not None
        polygon = tuple((point.x, point.y) for point in self._keepout.polygon.points)
        if not path_stays_outside_polygon(points, polygon):
            self._abort("Nav2 approach enters the remaining-cylinder keepout")
            return
        assert self._staging_pose is not None
        expected = self._staging_pose.pose.position
        if math.hypot(points[-1][0] - expected.x, points[-1][1] - expected.y) > self._float(
            "maximum_goal_error"
        ):
            self._abort("Nav2 approach endpoint does not match the staging pose")
            return
        self._checked_path_publisher.publish(path)
        self._send_navigation_goal(generation, path, length)

    def _send_navigation_goal(
        self, generation: int, checked_path: Path, length: float
    ) -> None:
        if generation != self._generation:
            return
        goal = FollowPath.Goal()
        goal.path = checked_path
        goal.controller_id = ""
        goal.goal_checker_id = self._string("goal_checker_id")
        future = self._follow_path.send_goal_async(goal)

        def done(completed) -> None:
            if generation != self._generation:
                return
            try:
                handle = completed.result()
            except Exception as error:
                self._abort(f"FollowPath request failed: {error}")
                return
            if not handle.accepted:
                self._abort("FollowPath goal rejected")
                return
            self._nav_goal_handle = handle
            self._navigation_started = self._now()
            self._tf_failure_count = 0
            self._state = "approaching"
            self._publish_status(
                "approaching", f"map-checked Nav2 path length={length:.3f}"
            )
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda result: self._navigation_result(result, generation)
            )

        future.add_done_callback(done)

    def _navigation_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        self._nav_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self._abort(f"NavigateToPose result failed: {error}")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"FollowPath status {wrapped.status}")
            return
        self._state = "approach_complete"
        self._stop_burst_remaining = 5
        self._publish_zero()
        self._publish_status(
            "approach_complete",
            "staging pose reached; pushing is not armed",
        )

    def _arm_push(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._service_failure(response, "execution_enabled is false")
        if self._state != "contact_complete":
            return self._service_failure(
                response, f"push requires contact_complete, got {self._state}"
            )
        fault = self._push_input_fault()
        if fault is not None:
            return self._service_failure(response, fault)
        pose = self._robot_pose()
        if pose is None:
            return self._service_failure(response, "robot transform unavailable")
        assert self._push_path is not None
        start = self._push_path.poses[0].pose.position
        if math.hypot(pose[0] - start.x, pose[1] - start.y) > 0.12:
            return self._service_failure(response, "robot is not at push-path start")
        if not self._target_present():
            return self._service_failure(response, "target not confirmed in fork corridor")
        self._state = "push_armed"
        self._armed_until = self._now() + self._float("arming_timeout")
        self._publish_status("push_armed", "call start_push before arm expires")
        response.success = True
        response.message = "low-speed push armed"
        return response

    def _arm_contact(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._service_failure(response, "execution_enabled is false")
        if self._state != "approach_complete":
            return self._service_failure(
                response, f"contact requires approach_complete, got {self._state}"
            )
        fault = self._push_input_fault()
        if fault is not None:
            return self._service_failure(response, fault)
        pose = self._robot_pose()
        if pose is None or self._staging_pose is None:
            return self._service_failure(response, "robot transform unavailable")
        staging = self._staging_pose.pose.position
        if math.hypot(pose[0] - staging.x, pose[1] - staging.y) > 0.12:
            return self._service_failure(response, "robot is not at pre-contact staging")
        if not self._target_present(self._float("contact_acquire_maximum_distance")):
            return self._service_failure(response, "target not visible in contact corridor")
        self._state = "contact_armed"
        self._armed_until = self._now() + self._float("arming_timeout")
        self._publish_status("contact_armed", "call start_contact before arm expires")
        response.success = True
        response.message = "low-speed contact armed"
        return response

    def _start_contact(self, request, response):
        del request
        if self._state != "contact_armed":
            return self._service_failure(
                response, f"contact is not armed: {self._state}"
            )
        if self._now() > self._armed_until:
            self._terminate("cancelled", "contact arming window expired")
            return self._service_failure(response, "contact arming window expired")
        pose = self._robot_pose()
        if pose is None:
            return self._service_failure(response, "robot transform unavailable")
        self._contact_origin = pose[:2]
        self._target_last_seen = self._now()
        self._navigation_started = self._now()
        self._state = "contacting"
        self._publish_status("contacting", "creeping from staging to fork contact")
        response.success = True
        response.message = "contact approach started"
        return response

    def _start_push(self, request, response):
        del request
        if self._state != "push_armed":
            return self._service_failure(response, f"push is not armed: {self._state}")
        if self._now() > self._armed_until:
            self._terminate("cancelled", "push arming window expired")
            return self._service_failure(response, "push arming window expired")
        if not self._target_present():
            return self._service_failure(response, "target not confirmed in fork corridor")
        self._push_progress = 0
        self._target_last_seen = self._now()
        self._navigation_started = self._now()
        self._state = "pushing"
        self._publish_status("pushing", "direct low-speed path tracking active")
        response.success = True
        response.message = "push started"
        return response

    def _arm_return(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._service_failure(response, "execution_enabled is false")
        if self._state != "push_complete":
            return self._service_failure(
                response, f"return requires push_complete, got {self._state}"
            )
        if self._return_path is None:
            return self._service_failure(response, "return path is missing")
        if self._now() - self._last_scan_received > self._float("scan_timeout"):
            return self._service_failure(response, "laser scan is missing or stale")
        if not self._compute_path.wait_for_server(timeout_sec=0.0):
            return self._service_failure(response, "ComputePathToPose is unavailable")
        if not self._follow_path.wait_for_server(timeout_sec=0.0):
            return self._service_failure(response, "FollowPath is unavailable")
        self._state = "return_armed"
        self._armed_until = self._now() + self._float("arming_timeout")
        self._publish_status("return_armed", "call start_return before arm expires")
        response.success = True
        response.message = "release and return armed"
        return response

    def _start_return(self, request, response):
        del request
        if self._state != "return_armed":
            return self._service_failure(response, f"return is not armed: {self._state}")
        if self._now() > self._armed_until:
            self._terminate("cancelled", "return arming window expired")
            return self._service_failure(response, "return arming window expired")
        pose = self._robot_pose()
        if pose is None:
            return self._service_failure(response, "robot transform unavailable")
        self._release_origin = pose
        self._release_previous_pose = pose
        self._navigation_started = self._now()
        self._state = "releasing"
        self._publish_status("releasing", "short monitored reverse from delivered target")
        response.success = True
        response.message = "release retreat started"
        return response

    def _finalize_delivery(self, request, response):
        del request
        if self._state != "return_complete":
            return self._service_failure(
                response, f"delivery requires return_complete, got {self._state}"
            )
        if self._snapshot is None:
            return self._service_failure(response, "locked inventory is unavailable")
        if self._plan_destination is None:
            return self._service_failure(response, "delivery destination is unavailable")
        clients = (
            self._plan_reset,
            self._validation_reset,
            self._snapshot_reset,
        )
        if sum(self._snapshot.expected_color_counts) > 1:
            clients += (
                self._exclusion_update,
                self._inventory_update,
                self._snapshot_start,
            )
        if not all(client.service_is_ready() for client in clients):
            return self._service_failure(response, "snapshot services are unavailable")
        try:
            colors, counts = inventory_after_delivery(
                self._snapshot.expected_colors,
                self._snapshot.expected_color_counts,
                self._plan_target_color,
            )
        except ValueError as error:
            return self._service_failure(response, str(error))
        self._generation += 1
        generation = self._generation
        next_exclusions = tuple(
            self._delivered_destinations + [self._plan_destination]
        )
        exclusion_radius = self._plan_exclusion_radius
        self._state = "finalizing"
        self._publish_status("finalizing", "resetting evidence for remaining inventory")
        first = self._plan_reset.call_async(Trigger.Request())
        first.add_done_callback(
            lambda done: self._after_plan_reset(
                done,
                generation,
                colors,
                counts,
                next_exclusions,
                exclusion_radius,
            )
        )
        response.success = True
        response.message = "remaining-inventory reset started"
        return response

    def _after_plan_reset(
        self, future, generation, colors, counts, exclusions, exclusion_radius
    ) -> None:
        if not self._service_result_ok(future, generation, "plan reset"):
            return
        future = self._validation_reset.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._after_validation_reset(
                done,
                generation,
                colors,
                counts,
                exclusions,
                exclusion_radius,
            )
        )

    def _after_validation_reset(
        self, future, generation, colors, counts, exclusions, exclusion_radius
    ) -> None:
        if not self._service_result_ok(future, generation, "validation reset"):
            return
        future = self._snapshot_reset.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._after_snapshot_reset(
                done,
                generation,
                colors,
                counts,
                exclusions,
                exclusion_radius,
            )
        )

    def _after_snapshot_reset(
        self, future, generation, colors, counts, exclusions, exclusion_radius
    ) -> None:
        if not self._service_result_ok(future, generation, "snapshot reset"):
            return
        if not any(counts):
            self._state = "task_complete"
            self._publish_status("task_complete", "all configured cylinders delivered")
            return
        request = SetSnapshotExclusions.Request()
        request.centers = [Point(x=x, y=y, z=0.0) for x, y in exclusions]
        request.radius = exclusion_radius
        future = self._exclusion_update.call_async(request)
        future.add_done_callback(
            lambda done: self._after_exclusion_update(
                done, generation, colors, counts, exclusions
            )
        )

    def _after_exclusion_update(
        self, future, generation, colors, counts, exclusions
    ) -> None:
        if not self._service_result_ok(future, generation, "exclusion update"):
            return
        self._delivered_destinations = list(exclusions)
        request = SetObjectInventory.Request()
        request.colors = list(colors)
        request.counts = list(counts)
        future = self._inventory_update.call_async(request)
        future.add_done_callback(
            lambda done: self._after_inventory_update(done, generation)
        )

    def _after_inventory_update(self, future, generation) -> None:
        if not self._service_result_ok(future, generation, "inventory update"):
            return
        future = self._snapshot_start.call_async(Trigger.Request())
        future.add_done_callback(
            lambda done: self._after_snapshot_start(done, generation)
        )

    def _after_snapshot_start(self, future, generation) -> None:
        if not self._service_result_ok(future, generation, "snapshot start"):
            return
        self._state = "collecting_remaining"
        self._publish_status(
            "collecting_remaining", "returned home; collecting reduced inventory"
        )

    def _service_result_ok(self, future, generation: int, label: str) -> bool:
        if generation != self._generation:
            return False
        try:
            result = future.result()
        except Exception as error:
            self._abort(f"{label} failed: {error}")
            return False
        if not result.success:
            self._abort(f"{label} rejected: {result.message}")
            return False
        return True

    def _cancel(self, request, response):
        del request
        if self._state in {"idle", "cancelled", "failed"}:
            return self._service_failure(response, "no active approach")
        self._terminate("cancelled", "cancel requested")
        response.success = True
        response.message = "approach cancelled; zero velocity requested"
        return response

    def _reset(self, request, response):
        del request
        self._terminate("idle", "reset")
        self._plan_ready = False
        self._plan_stamp = None
        self._announced_plan_stamp = None
        self._snapshot_ready = False
        self._snapshot_locked = False
        self._staging_pose = None
        self._approach_preview = None
        self._target_path = None
        self._push_path = None
        self._return_path = None
        self._keepout = None
        self._snapshot = None
        self._plan_target_id = -1
        self._plan_target_color = ""
        self._plan_destination = None
        self._plan_exclusion_radius = 0.0
        self._release_origin = None
        self._release_previous_pose = None
        self._checked_path_publisher.publish(Path())
        response.success = True
        response.message = "approach executor reset; delivery history preserved"
        return response

    def _reset_task(self, request, response):
        response = self._reset(request, response)
        self._delivered_destinations = []
        self._publish_status("idle", "new task reset; delivery history cleared")
        response.message = "approach executor and delivery history reset"
        return response

    def _tick(self) -> None:
        if self._stop_burst_remaining > 0:
            self._publish_zero()
            self._stop_burst_remaining -= 1
        now = self._now()
        if self._state in {
            "armed",
            "contact_armed",
            "push_armed",
            "return_armed",
        } and now > self._armed_until:
            self._terminate("cancelled", "arming window expired")
            return
        if self._state == "contacting":
            self._tick_contact(now)
            return
        if self._state == "pushing":
            self._tick_push(now)
            return
        if self._state == "releasing":
            self._tick_release(now)
            return
        if self._state == "returning":
            if now - self._last_scan_received > self._float("scan_timeout"):
                self._abort("laser scan became stale during return")
            elif now - self._navigation_started > self._float("return_timeout"):
                self._abort("return navigation timed out")
            else:
                pose = self._robot_pose()
                if pose is None:
                    self._tf_failure_count += 1
                    if self._tf_failure_count >= 3:
                        self._abort("robot transform lost during return")
                else:
                    self._tf_failure_count = 0
                    assert self._keepout is not None
                    polygon = tuple(
                        (point.x, point.y)
                        for point in self._keepout.polygon.points
                    )
                    if not point_outside_polygon(pose[:2], polygon):
                        self._abort("robot entered remaining keepout during return")
            return
        if self._state != "approaching":
            return
        fault = approach_watchdog_fault(
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
            assert self._keepout is not None
            polygon = tuple(
                (point.x, point.y) for point in self._keepout.polygon.points
            )
            position = transform.transform.translation
            if not point_outside_polygon((position.x, position.y), polygon):
                self._abort("robot entered the remaining-cylinder keepout")
        except TransformException:
            self._tf_failure_count += 1
            if self._tf_failure_count >= 3:
                self._abort("robot transform lost during approach")

    def _tick_contact(self, now: float) -> None:
        if now - self._last_scan_received > self._float("scan_timeout"):
            self._abort("laser scan became stale during contact")
            return
        if now - self._navigation_started > self._float("contact_timeout"):
            self._abort("contact approach timed out")
            return
        acquire_distance = self._float("contact_acquire_maximum_distance")
        if self._target_present(acquire_distance):
            self._target_last_seen = now
        elif now - self._target_last_seen > self._float("target_loss_timeout"):
            self._abort("target was lost during contact approach")
            return
        if self._unexpected_front_obstacle(acquire_distance):
            self._abort("unexpected obstacle entered contact corridor")
            return
        pose = self._robot_pose()
        if pose is None or self._contact_origin is None:
            self._tf_failure_count += 1
            if self._tf_failure_count >= 3:
                self._abort("robot transform lost during contact")
            return
        self._tf_failure_count = 0
        assert self._staging_pose is not None
        assert self._push_path is not None
        start = self._staging_pose.pose.position
        end = self._push_path.poses[0].pose.position
        points = ((start.x, start.y), (end.x, end.y))
        assert self._keepout is not None
        polygon = tuple((point.x, point.y) for point in self._keepout.polygon.points)
        if not point_outside_polygon(pose[:2], polygon):
            self._abort("robot entered remaining-cylinder keepout during contact")
            return
        if math.dist(pose[:2], self._contact_origin) > 1.5 * math.dist(*points):
            self._abort("contact travel exceeded planned distance")
            return
        if math.dist(pose[:2], points[-1]) <= self._float("contact_goal_tolerance"):
            if not self._target_present():
                self._abort("planned contact reached without target in fork corridor")
                return
            self._state = "contact_complete"
            self._stop_burst_remaining = 5
            self._publish_zero()
            self._publish_status(
                "contact_complete", "fork contact confirmed; push is not armed"
            )
            return
        linear, angular, _progress, _error = tracking_command(
            points,
            pose[0],
            pose[1],
            pose[2],
            0,
            0.08,
            self._float("contact_speed"),
            self._float("push_angular_gain"),
            self._float("contact_maximum_angular_speed"),
        )
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._stop_publisher.publish(command)

    def _tick_push(self, now: float) -> None:
        if now - self._last_scan_received > self._float("scan_timeout"):
            self._abort("laser scan became stale during push")
            return
        if now - self._navigation_started > self._float("push_timeout"):
            self._abort("push timed out")
            return
        if self._target_present():
            self._target_last_seen = now
        elif now - self._target_last_seen > self._float("target_loss_timeout"):
            self._abort("pushed target was lost from fork corridor")
            return
        if self._unexpected_front_obstacle():
            self._abort("unexpected obstacle entered push corridor")
            return
        pose = self._robot_pose()
        if pose is None:
            self._tf_failure_count += 1
            if self._tf_failure_count >= 3:
                self._abort("robot transform lost during push")
            return
        self._tf_failure_count = 0
        assert self._push_path is not None
        points = tuple(
            (item.pose.position.x, item.pose.position.y)
            for item in self._push_path.poses
        )
        endpoint = points[-1]
        assert self._keepout is not None
        polygon = tuple((point.x, point.y) for point in self._keepout.polygon.points)
        if not point_outside_polygon(pose[:2], polygon):
            self._abort("robot entered remaining-cylinder keepout during push")
            return
        path_error = min(math.dist(pose[:2], point) for point in points)
        if path_error > self._float("maximum_push_deviation"):
            self._abort(f"push path deviation is {path_error:.3f} m")
            return
        if math.dist(pose[:2], endpoint) <= self._float("push_goal_tolerance"):
            self._state = "push_complete"
            self._stop_burst_remaining = 5
            self._publish_zero()
            self._publish_status(
                "push_complete", "destination reached; arm return explicitly"
            )
            return
        linear, angular, progress, _error = tracking_command(
            points,
            pose[0],
            pose[1],
            pose[2],
            self._push_progress,
            self._float("push_lookahead"),
            self._float("push_speed"),
            self._float("push_angular_gain"),
            self._float("push_maximum_angular_speed"),
        )
        self._push_progress = progress
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._stop_publisher.publish(command)

    def _tick_release(self, now: float) -> None:
        if now - self._last_scan_received > self._float("scan_timeout"):
            self._abort("laser scan became stale during release")
            return
        if now - self._navigation_started > self._float("release_timeout"):
            self._abort("release retreat timed out")
            return
        if self._rear_obstacle():
            self._abort("obstacle behind robot during release")
            return
        pose = self._robot_pose()
        if (
            pose is None
            or self._release_origin is None
            or self._release_previous_pose is None
        ):
            self._tf_failure_count += 1
            if self._tf_failure_count >= 3:
                self._abort("robot transform lost during release")
            return
        self._tf_failure_count = 0
        pose_step = retreat_pose_step(
            self._release_previous_pose[0],
            self._release_previous_pose[1],
            pose[0],
            pose[1],
        )
        if pose_step > self._float("maximum_release_pose_step"):
            self._abort(f"release pose jumped by {pose_step:.3f} m")
            return
        self._release_previous_pose = pose
        reverse_progress, lateral_drift, heading_error = retreat_motion(
            *self._release_origin,
            *pose,
        )
        if reverse_progress < -0.02:
            self._abort("robot moved forward during release retreat")
            return
        if abs(lateral_drift) > self._float(
            "maximum_release_lateral_deviation"
        ):
            self._abort(
                f"release lateral drift is {abs(lateral_drift):.3f} m"
            )
            return
        if abs(heading_error) > self._float("maximum_release_heading_error"):
            self._abort(
                f"release heading error is {abs(heading_error):.3f} rad"
            )
            return
        if reverse_progress >= self._float("release_distance"):
            self._publish_zero()
            self._begin_return_path_check()
            return
        command = Twist()
        command.linear.x = -self._float("release_speed")
        self._stop_publisher.publish(command)

    def _begin_return_path_check(self) -> None:
        assert self._return_path is not None
        self._generation += 1
        generation = self._generation
        final = self._return_path.poses[-1]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self._return_path.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = final.pose
        self._return_home = goal_pose
        self._state = "checking_return"
        self._publish_status("checking_return", "requesting collision-checked home path")
        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.use_start = False
        future = self._compute_path.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._return_compute_response(done, generation)
        )

    def _return_compute_response(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._abort(f"return ComputePathToPose request failed: {error}")
            return
        if not handle.accepted:
            self._abort("return ComputePathToPose goal rejected")
            return
        self._compute_goal_handle = handle
        future = handle.get_result_async()
        future.add_done_callback(
            lambda done: self._return_compute_result(done, generation)
        )

    def _return_compute_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        self._compute_goal_handle = None
        try:
            wrapped = future.result()
            path = wrapped.result.path
            error_code = nav2_path_result_error_code(wrapped.result)
        except Exception as error:
            self._abort(f"return path result failed: {error}")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or error_code != 0:
            self._abort(f"return planning failed with code {error_code}")
            return
        points = tuple((pose.pose.position.x, pose.pose.position.y) for pose in path.poses)
        if len(points) < 2:
            self._abort("Nav2 returned an empty return path")
            return
        length = path_length(points)
        if length > self._float("maximum_return_length"):
            self._abort(f"Nav2 return is too long: {length:.3f} m")
            return
        assert self._keepout is not None
        polygon = tuple((point.x, point.y) for point in self._keepout.polygon.points)
        if not path_stays_outside_polygon(points, polygon):
            self._abort("Nav2 return path enters remaining-cylinder keepout")
            return
        self._checked_path_publisher.publish(path)
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = ""
        goal.goal_checker_id = self._string("goal_checker_id")
        future = self._follow_path.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._return_navigation_response(done, generation)
        )

    def _return_navigation_response(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._abort(f"return FollowPath request failed: {error}")
            return
        if not handle.accepted:
            self._abort("return FollowPath goal rejected")
            return
        self._nav_goal_handle = handle
        self._navigation_started = self._now()
        self._state = "returning"
        self._publish_status("returning", "following collision-checked home path")
        future = handle.get_result_async()
        future.add_done_callback(
            lambda done: self._return_navigation_result(done, generation)
        )

    def _return_navigation_result(self, future, generation: int) -> None:
        if generation != self._generation:
            return
        self._nav_goal_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            self._abort(f"return FollowPath result failed: {error}")
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._abort(f"return FollowPath status {wrapped.status}")
            return
        self._state = "return_complete"
        self._stop_burst_remaining = 5
        self._publish_zero()
        self._publish_status(
            "return_complete", "home reached; finalize delivery explicitly"
        )

    def _input_fault(self) -> str | None:
        fault = plan_inputs_fault(
            self._plan_ready,
            self._snapshot_ready,
            self._snapshot_locked,
            self._staging_pose is not None,
            self._approach_preview is not None,
            self._keepout is not None,
            self._push_path is not None,
            self._return_path is not None,
            self._target_path is not None,
        )
        if fault is not None:
            return fault
        assert self._staging_pose is not None
        assert self._approach_preview is not None
        assert self._push_path is not None
        assert self._return_path is not None
        assert self._target_path is not None
        assert self._keepout is not None
        fixed_frame = self._string("fixed_frame")
        messages = (
            self._staging_pose,
            self._approach_preview,
            self._target_path,
            self._push_path,
            self._return_path,
            self._keepout,
        )
        if any(message.header.frame_id != fixed_frame for message in messages):
            return "plan inputs do not use the configured fixed frame"
        stamps = {
            (message.header.stamp.sec, message.header.stamp.nanosec)
            for message in messages
        }
        if len(stamps) != 1:
            return "plan inputs belong to different preview generations"
        identity_fault = self._selected_target_fault()
        if identity_fault is not None:
            return identity_fault
        assert self._plan_destination is not None
        target_endpoint = self._target_path.poses[-1].pose.position
        if math.dist(
            (target_endpoint.x, target_endpoint.y), self._plan_destination
        ) > 0.01:
            return "target path endpoint disagrees with delivery destination"
        return None

    def _push_input_fault(self) -> str | None:
        if (
            self._snapshot is None
            or not self._snapshot.ready
            or not self._snapshot.locked
        ):
            return "validated cylinder snapshot is not ready and locked"
        if self._push_path is None or self._return_path is None or self._keepout is None:
            return "push, return, or keepout plan is missing"
        if self._last_scan is None:
            return "laser scan is missing"
        if self._now() - self._last_scan_received > self._float("scan_timeout"):
            return "laser scan is stale"
        fixed_frame = self._string("fixed_frame")
        messages = (self._push_path, self._return_path, self._keepout)
        if any(message.header.frame_id != fixed_frame for message in messages):
            return "push plan does not use the configured fixed frame"
        stamps = {
            (message.header.stamp.sec, message.header.stamp.nanosec)
            for message in messages
        }
        if len(stamps) != 1:
            return "push plan inputs belong to different preview generations"
        return self._selected_target_fault()

    def _selected_target_fault(self) -> str | None:
        if self._snapshot is None:
            return "validated cylinder snapshot is missing"
        target = next(
            (
                item
                for item in self._snapshot.objects
                if item.candidate_id == self._plan_target_id
            ),
            None,
        )
        if target is None or target.state != "validated":
            return "selected target is absent from locked validation"
        if target.color != self._plan_target_color:
            return "selected target color disagrees with push preview"
        return None

    def _robot_pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._string("fixed_frame"),
                self._string("robot_frame"),
                Time(),
                timeout=Duration(seconds=0.02),
            ).transform
        except TransformException:
            return None
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        return transform.translation.x, transform.translation.y, yaw

    def _target_present(self, maximum_distance: float | None = None) -> bool:
        if self._last_scan is None:
            return False
        scan = self._last_scan
        return front_target_present(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            maximum_distance
            if maximum_distance is not None
            else self._float("target_maximum_distance"),
            self._float("target_corridor_half_width"),
        )

    def _unexpected_front_obstacle(
        self, target_corridor_depth: float | None = None
    ) -> bool:
        if self._last_scan is None:
            return True
        scan = self._last_scan
        return unexpected_obstacle_ahead(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._float("obstacle_stop_distance"),
            self._float("obstacle_half_width"),
            self._float("target_corridor_half_width"),
            target_corridor_depth
            if target_corridor_depth is not None
            else self._float("target_maximum_distance"),
        )

    def _rear_obstacle(self) -> bool:
        if self._last_scan is None:
            return True
        scan = self._last_scan
        return obstacle_behind(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._float("rear_stop_distance"),
            self._float("obstacle_half_width"),
        )

    def _copy_staging_with_current_stamp(self) -> PoseStamped:
        assert self._staging_pose is not None
        output = PoseStamped()
        output.header.frame_id = self._staging_pose.header.frame_id
        output.header.stamp = self.get_clock().now().to_msg()
        output.pose = self._staging_pose.pose
        return output

    def _accept_plan_updates(self) -> bool:
        return self._state in {
            "idle",
            "failed",
            "cancelled",
            "collecting_remaining",
        }

    def _terminate(self, state: str, reason: str) -> None:
        self._generation += 1
        if self._compute_goal_handle is not None:
            self._compute_goal_handle.cancel_goal_async()
            self._compute_goal_handle = None
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._state = state
        self._armed_until = 0.0
        self._stop_burst_remaining = 5
        self._publish_zero()
        self._publish_status(state, reason)

    def _abort(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._terminate("failed", reason)

    def _service_failure(self, response, reason: str):
        response.success = False
        response.message = reason
        return response

    def _publish_zero(self) -> None:
        self._stop_publisher.publish(Twist())

    def _publish_status(self, state: str, reason: str) -> None:
        payload = {
            "execution_enabled": self._bool("execution_enabled"),
            "map_collision_checked": state
            in {
                "approaching",
                "approach_complete",
                "returning",
                "return_complete",
                "finalizing",
                "collecting_remaining",
            },
            "remaining_keepout_checked": state
            in {
                "approaching",
                "approach_complete",
                "push_armed",
                "pushing",
                "push_complete",
                "return_armed",
                "releasing",
                "checking_return",
                "returning",
                "return_complete",
                "finalizing",
                "collecting_remaining",
            },
            "reason": reason,
            "state": state,
            "target_id": self._plan_target_id,
            "target_color": self._plan_target_color,
            "delivery_destination": self._plan_destination,
            "delivered_exclusion_count": len(self._delivered_destinations),
        }
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def prepare_shutdown(self) -> None:
        """Cancel actions and request zero velocity before ROS shutdown."""

        self._generation += 1
        if self._compute_goal_handle is not None:
            self._compute_goal_handle.cancel_goal_async()
            self._compute_goal_handle = None
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

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ApproachExecutionNode()
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
