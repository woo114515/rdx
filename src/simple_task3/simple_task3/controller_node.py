"""A small map-frame controller for the complete Task 3 cycle."""

from __future__ import annotations

import json
import math
import signal
import threading
import time

import rclpy
from color_object_sorter_interfaces.msg import ValidatedCylinderArray
from color_object_sorter_interfaces.srv import (
    SetObjectInventory,
    SetSnapshotExclusions,
)
from cylinder_push_planner.direct_cycle import (
    DirectCyclePlan,
    build_direct_cycle,
    enforce_minimum_linear_speed,
    heading_error,
    pure_pursuit_command,
    transform_point_2d,
)
from cylinder_push_planner.execution_safety import (
    front_target_present,
    obstacle_behind,
    retreat_motion,
    unexpected_obstacle_ahead,
)
from cylinder_push_planner.geometry import (
    Target,
    destination_slot,
    local_offset_to_map,
    select_right_first,
)
from cylinder_push_planner.inventory_cycle import inventory_after_delivery
from cylinder_push_planner.task_workflow import (
    snapshot_matches_collection,
    validate_inventory,
)
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
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
from visualization_msgs.msg import Marker, MarkerArray

from .route import tracking_target


ACTIVE_STATES = {
    "align_approach",
    "approach",
    "align_contact",
    "contact",
    "push",
    "release",
    "align_return",
    "return",
    "align_home",
}
COLLECTION_STATES = {"collecting", "collecting_remaining"}


class SimpleTaskController(Node):
    """Follow one immutable map-frame route and optionally repeat for inventory."""

    def __init__(self) -> None:
        super().__init__("simple_task_controller")
        defaults = {
            "execution_enabled": False,
            "path_only_mode": False,
            "fixed_frame": "map",
            "robot_frame": "base_footprint",
            "snapshot_topic": "/cylinder_snapshot/validated_objects",
            "scan_topic": "/scan",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "cmd_vel_topic": "/cmd_vel",
            "require_fixed_localization": True,
            "localization_status_topic": "/simple_task3/localization_status",
            "inventory_colors": ["blue", "green", "red"],
            "inventory_counts": [2, 2, 2],
            "destination_colors": ["blue", "green", "red"],
            "destination_slot_spacing": 0.18,
            "staging_clearance": 0.25,
            "contact_offset": 0.15,
            "release_distance": 0.15,
            "transit_clearance": 0.10,
            "delivered_exclusion_radius": 0.18,
            "maximum_route_radius": 3.0,
            "control_rate": 10.0,
            "minimum_linear_speed": 0.15,
            "approach_speed": 0.15,
            "contact_speed": 0.15,
            "push_speed": 0.15,
            "release_speed": 0.15,
            "return_speed": 0.15,
            "angular_gain": 3.0,
            "maximum_angular_speed": 0.50,
            "heading_tolerance": 0.08,
            "drive_heading_limit": 0.60,
            "path_lookahead": 0.30,
            "maximum_segment_advance": 12,
            "staging_tolerance": 0.05,
            "contact_tolerance": 0.035,
            "push_tolerance": 0.06,
            "home_tolerance": 0.08,
            "maximum_path_deviation": 0.12,
            "maximum_home_error_before_cycle": 0.30,
            "maximum_map_position_step": 0.08,
            "maximum_map_yaw_step": 0.18,
            "scan_timeout": 0.75,
            "motion_scan_timeout": 1.0,
            "odom_timeout": 0.75,
            "message_stamp_timeout": 1.0,
            "transform_timeout": 0.10,
            "transform_failure_limit": 3,
            "alignment_timeout": 15.0,
            "approach_timeout": 40.0,
            "contact_timeout": 15.0,
            "push_timeout": 60.0,
            "release_timeout": 10.0,
            "return_timeout": 50.0,
            "snapshot_wait_timeout": 120.0,
            "home_settle_cycles": 3,
            "target_acquire_distance": 0.65,
            "target_maximum_distance": 0.32,
            "target_corridor_half_width": 0.07,
            "live_obstacle_stop_enabled": True,
            "obstacle_stop_distance": 0.32,
            "obstacle_half_width": 0.18,
            "rear_stop_distance": 0.25,
            "release_lateral_limit": 0.05,
            "release_heading_limit": 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        destination_defaults = {
            "blue": (0.0, 2.0),
            "green": (2.0, 0.0),
            "red": (0.0, -2.0),
        }
        for color in self.get_parameter("destination_colors").value:
            default = destination_defaults.get(str(color))
            for axis, index in (("x", 0), ("y", 1)):
                name = f"destinations.{color}.{axis}"
                if default is None:
                    self.declare_parameter(name, Parameter.Type.DOUBLE)
                else:
                    self.declare_parameter(name, default[index])
        self._validate_parameters()

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            String, "/simple_task3/status", latched
        )
        self._path_pub = self.create_publisher(Path, "/simple_task3/route", latched)
        self._marker_pub = self.create_publisher(
            MarkerArray, "/simple_task3/markers", latched
        )
        self._cmd_pub = self.create_publisher(
            Twist, self._string("cmd_vel_topic"), 10
        )

        self._sensor_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            ValidatedCylinderArray,
            self._string("snapshot_topic"),
            self._on_snapshot,
            latched,
        )
        self.create_subscription(
            LaserScan,
            self._string("scan_topic"),
            self._on_scan,
            qos_profile_sensor_data,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            Odometry,
            self._string("odom_topic"),
            self._on_odom,
            20,
            callback_group=self._sensor_group,
        )
        self.create_subscription(
            String,
            self._string("localization_status_topic"),
            self._on_localization_status,
            latched,
        )
        self.create_service(Trigger, "/simple_task3/prepare", self._prepare)
        self.create_service(Trigger, "/simple_task3/run_once", self._run_once)
        self.create_service(Trigger, "/simple_task3/run_all", self._run_all)
        self.create_service(Trigger, "/simple_task3/cancel", self._cancel)

        self._validation_reset = self.create_client(
            Trigger, "/cylinder_validation/reset"
        )
        self._snapshot_reset = self.create_client(
            Trigger, "/cylinder_snapshot/reset"
        )
        self._snapshot_start = self.create_client(
            Trigger, "/cylinder_snapshot/start"
        )
        self._inventory_update = self.create_client(
            SetObjectInventory, "/cylinder_snapshot/set_inventory"
        )
        self._exclusion_update = self.create_client(
            SetSnapshotExclusions, "/cylinder_snapshot/set_exclusions"
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state = "idle"
        self._reason = "call /simple_task3/prepare or /simple_task3/run_all"
        self._snapshot: ValidatedCylinderArray | None = None
        self._expected_inventory: tuple[tuple[str, ...], tuple[int, ...]] | None = None
        self._collection_started = 0.0
        self._scan: LaserScan | None = None
        self._scan_received = 0.0
        self._scan_stamp = -1.0
        self._scan_stamp_reversed = False
        self._odom_pose: tuple[float, float, float] | None = None
        self._odom_received = 0.0
        self._odom_stamp = -1.0
        self._odom_stamp_reversed = False
        self._plan: DirectCyclePlan | None = None
        self._cursor = 0
        self._phase_started = 0.0
        self._release_origin: tuple[float, float, float] | None = None
        self._task_home: tuple[float, float, float] | None = None
        self._field_center: tuple[float, float] | None = None
        self._initial_counts: dict[str, int] = {}
        self._delivered_destinations: list[tuple[float, float]] = []
        self._previous_map_pose: tuple[float, float, float] | None = None
        self._previous_map_pose_time = 0.0
        self._tf_failures = 0
        self._home_settle = 0
        self._run_all_active = False
        self._operation_busy = False
        self._fixed_localization_ready = False
        self._localization_owner = "unknown"
        self._generation = 0
        self._stop_burst = 0
        self.create_timer(1.0 / self._float("control_rate"), self._tick)
        self._publish_status()
        self.get_logger().info("Simple Task 3 controller ready; motion is gated")

    def _on_snapshot(self, message: ValidatedCylinderArray) -> None:
        if self._state not in COLLECTION_STATES or self._expected_inventory is None:
            return
        stamp = self._stamp_seconds(message.header.stamp)
        colors, counts = self._expected_inventory
        if snapshot_matches_collection(
            colors,
            counts,
            message.expected_colors,
            message.expected_color_counts,
            stamp,
            self._collection_started,
        ):
            self._snapshot = message

    def _on_scan(self, message: LaserScan) -> None:
        stamp = self._stamp_seconds(message.header.stamp)
        if self._scan_stamp >= 0.0 and stamp < self._scan_stamp:
            self._scan_stamp_reversed = True
        self._scan_stamp = stamp
        self._scan = message
        self._scan_received = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        stamp = self._stamp_seconds(message.header.stamp)
        if self._odom_stamp >= 0.0 and stamp < self._odom_stamp:
            self._odom_stamp_reversed = True
        self._odom_stamp = stamp
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        position = message.pose.pose.position
        self._odom_pose = (position.x, position.y, yaw)
        self._odom_received = time.monotonic()

    def _on_localization_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self._fixed_localization_ready = bool(payload.get("ready", False))
        self._localization_owner = str(payload.get("localization_owner", "unknown"))

    def _prepare(self, request, response):
        del request
        return self._start_new_task(response, run_all=False)

    def _run_all(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._failure(response, "execution_enabled is false")
        return self._start_new_task(response, run_all=True)

    def _start_new_task(self, response, *, run_all: bool):
        if self._state in ACTIVE_STATES or self._operation_busy:
            return self._failure(response, f"controller is busy: {self._state}")
        colors = [str(value) for value in self.get_parameter("inventory_colors").value]
        counts = [int(value) for value in self.get_parameter("inventory_counts").value]
        try:
            validate_inventory(colors, counts)
        except ValueError as error:
            return self._failure(response, str(error))
        if not self._services_ready():
            return self._failure(response, "perception services are unavailable")

        self._run_all_active = run_all
        self._terminate("preparing", "resetting perception", clear_plan=True)
        self._task_home = None
        self._field_center = None
        self._initial_counts = dict(zip(colors, counts))
        self._delivered_destinations.clear()
        self._expected_inventory = (tuple(colors), tuple(counts))
        exclusions = SetSnapshotExclusions.Request()
        exclusions.centers = []
        exclusions.radius = 0.0
        inventory = SetObjectInventory.Request()
        inventory.colors = colors
        inventory.counts = counts
        self._run_steps(
            [
                ("validation reset", self._validation_reset, Trigger.Request()),
                ("snapshot reset", self._snapshot_reset, Trigger.Request()),
                ("exclusions clear", self._exclusion_update, exclusions),
                ("inventory update", self._inventory_update, inventory),
                ("snapshot start", self._snapshot_start, Trigger.Request()),
            ],
            "collecting",
            "waiting for a locked validated inventory",
        )
        response.success = True
        response.message = "run_all accepted" if run_all else "preparation accepted"
        return response

    def _run_once(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._failure(response, "execution_enabled is false")
        if self._run_all_active:
            return self._failure(response, "run_all is active")
        if self._state in ACTIVE_STATES or self._operation_busy:
            return self._failure(response, f"controller is busy: {self._state}")
        try:
            message = self._begin_cycle()
        except ValueError as error:
            return self._failure(response, str(error))
        response.success = True
        response.message = message
        return response

    def _cancel(self, request, response):
        del request
        self._generation += 1
        self._operation_busy = False
        self._run_all_active = False
        self._terminate("cancelled", "operator requested cancellation")
        response.success = True
        response.message = "cancelled; zero velocity requested"
        return response

    def _begin_cycle(self) -> str:
        snapshot = self._snapshot
        if snapshot is None or not snapshot.ready or not snapshot.locked:
            raise ValueError("validated snapshot is not ready and locked")
        if self._bool("require_fixed_localization") and not self._fixed_localization_ready:
            raise ValueError("fixed-map AMCL localization is not ready")
        if not self._exclusive_command_owner():
            raise ValueError("/cmd_vel has another publisher")
        pose = self._robot_pose(self._string("fixed_frame"))
        if pose is None:
            raise ValueError("map-frame robot pose is unavailable")
        plan = self._make_plan(pose, snapshot)
        self._plan = plan
        self._publish_plan(plan)
        self._activate_plan(pose)
        return (
            f"cycle accepted for candidate {plan.selection.target.candidate_id} "
            f"({plan.selection.target.color})"
        )

    def _activate_plan(self, pose: tuple[float, float, float]) -> None:
        """Begin following only after GMapping confirms map rebuilds are paused."""
        self._previous_map_pose = pose
        self._previous_map_pose_time = time.monotonic()
        self._tf_failures = 0
        self._set_state("align_approach", "aligning with the planned route")

    def _make_plan(
        self,
        pose: tuple[float, float, float],
        snapshot: ValidatedCylinderArray,
    ) -> DirectCyclePlan:
        fixed_frame = self._string("fixed_frame")
        source_frame = snapshot.header.frame_id or fixed_frame
        translation, rotation = self._planar_transform(fixed_frame, source_frame)
        targets = tuple(
            Target(
                candidate_id=item.candidate_id,
                x=position[0],
                y=position[1],
                color=item.color,
                radius=item.radius,
            )
            for item in snapshot.objects
            if item.state == "validated"
            for position in (
                transform_point_2d(
                    (item.position.x, item.position.y), translation, rotation
                ),
            )
        )
        expected = sum(int(value) for value in snapshot.expected_color_counts)
        if len(targets) != expected:
            raise ValueError(
                f"snapshot has {len(targets)} validated targets; expected {expected}"
            )
        selection = select_right_first(
            targets, pose[0], pose[1], self._float("staging_clearance")
        )
        home = self._task_home if self._task_home is not None else pose
        if (
            not self._bool("path_only_mode")
            and self._task_home is not None
            and math.dist(pose[:2], home[:2])
            > self._float("maximum_home_error_before_cycle")
        ):
            raise ValueError("robot is not near the saved task home")
        field_center = self._field_center or (
            selection.envelope_x,
            selection.envelope_y,
        )
        destination = self._destination(
            selection.target.color, snapshot, field_center, home
        )
        plan = build_direct_cycle(
            targets,
            pose,
            destination,
            self._float("staging_clearance"),
            self._float("contact_offset"),
            self._float("release_distance"),
            self._float("transit_clearance"),
            selection=selection,
            home=home,
            field_center=field_center,
            # path_only_mode controls execution-time vetoes, not planning.
            # Generate the immutable route around the remaining-cylinder
            # keepout before motion starts, then follow that route as-is.
            require_clear_corridors=True,
        )
        self._validate_route_radius(plan)
        self._task_home = home
        self._field_center = field_center
        return plan

    def _destination(
        self,
        color: str,
        snapshot: ValidatedCylinderArray,
        field_center: tuple[float, float],
        home: tuple[float, float, float],
    ) -> tuple[float, float]:
        colors = tuple(str(value) for value in snapshot.expected_colors)
        counts = tuple(int(value) for value in snapshot.expected_color_counts)
        current = dict(zip(colors, counts))
        initial = self._initial_counts.get(color)
        if initial is None or current.get(color, 0) < 1:
            raise ValueError(f"invalid current inventory for {color!r}")
        local_x = self._float(f"destinations.{color}.x")
        local_y = self._float(f"destinations.{color}.y")
        local_x, local_y = destination_slot(
            local_x,
            local_y,
            initial - current[color],
            initial,
            self._float("destination_slot_spacing"),
        )
        return local_offset_to_map(
            field_center[0], field_center[1], home[2], local_x, local_y
        )

    def _validate_route_radius(self, plan: DirectCyclePlan) -> None:
        center = plan.field_center
        maximum = self._float("maximum_route_radius")
        points = (
            *plan.approach_path,
            *plan.contact_path,
            *plan.push_path,
            *plan.return_path,
        )
        if any(math.dist(point, center) > maximum for point in points):
            raise ValueError("planned route leaves the configured task radius")

    def _tick(self) -> None:
        if self._stop_burst > 0:
            self._publish_zero()
            self._stop_burst -= 1
        if self._state not in ACTIVE_STATES:
            self._tick_automation()
            return
        pose = self._robot_pose(self._string("fixed_frame"))
        if pose is None:
            # Continuing to publish the previous non-zero command without a
            # pose cannot constitute path following. Stop immediately while
            # allowing a few TF cycles to recover before declaring failure.
            self._publish_zero()
            self._tf_failures += 1
            if self._tf_failures >= self._int("transform_failure_limit"):
                self._abort("map-frame robot transform was lost")
            return
        self._tf_failures = 0
        if not self._bool("path_only_mode") and self._map_pose_jumped(pose):
            self._abort("map-frame pose changed discontinuously")
            return
        assert self._plan is not None

        if self._state == "align_approach":
            self._align(pose, self._plan.approach_path, "approach")
        elif self._state == "approach":
            if self._front_obstacle():
                self._abort("obstacle entered the approach route")
            else:
                self._follow(
                    pose,
                    self._plan.approach_path,
                    self._float("approach_speed"),
                    self._float("staging_tolerance"),
                    self._float("approach_timeout"),
                    "align_contact",
                )
        elif self._state == "align_contact":
            self._align(
                pose,
                self._plan.contact_path,
                "contact",
                require_target=not self._bool("path_only_mode"),
            )
        elif self._state == "contact":
            if self._unexpected_front_obstacle(
                self._float("target_acquire_distance")
            ):
                self._abort("unexpected obstacle entered the contact route")
            else:
                self._follow(
                    pose,
                    self._plan.contact_path,
                    self._float("contact_speed"),
                    self._float("contact_tolerance"),
                    self._float("contact_timeout"),
                    "push",
                    allow_rotate_in_place=self._bool("path_only_mode"),
                )
        elif self._state == "push":
            if self._unexpected_front_obstacle(
                self._float("target_maximum_distance")
            ):
                self._abort("unexpected obstacle entered the push route")
            else:
                before = self._state
                self._follow(
                    pose,
                    self._plan.push_path,
                    self._float("push_speed"),
                    self._float("push_tolerance"),
                    self._float("push_timeout"),
                    "release",
                    allow_rotate_in_place=self._bool("path_only_mode"),
                )
                if before != self._state and self._state == "release":
                    if self._odom_pose is None:
                        self._abort("odometry is unavailable at release")
                    else:
                        self._release_origin = self._odom_pose
        elif self._state == "release":
            self._tick_release()
        elif self._state == "align_return":
            self._align(pose, self._plan.return_path, "return")
        elif self._state == "return":
            if self._front_obstacle():
                self._abort("obstacle entered the return route")
            else:
                self._follow(
                    pose,
                    self._plan.return_path,
                    self._float("return_speed"),
                    self._float("home_tolerance"),
                    self._float("return_timeout"),
                    "align_home",
                )
        elif self._state == "align_home":
            self._align_home(pose)

    def _tick_automation(self) -> None:
        if not self._run_all_active or self._operation_busy:
            return
        if self._state not in COLLECTION_STATES:
            return
        if time.monotonic() - self._phase_started > self._float(
            "snapshot_wait_timeout"
        ):
            self._abort("timed out waiting for a locked snapshot")
            return
        snapshot = self._snapshot
        if snapshot is None or not snapshot.ready or not snapshot.locked:
            return
        if self._bool("require_fixed_localization") and not self._fixed_localization_ready:
            self._reason = "waiting for fixed-map AMCL localization"
            self._publish_status()
            return
        try:
            self._begin_cycle()
        except ValueError as error:
            self._abort(f"automatic cycle could not start: {error}")

    def _align(
        self,
        pose: tuple[float, float, float],
        path,
        next_state: str,
        *,
        require_target: bool = False,
    ) -> None:
        if (
            not self._bool("path_only_mode")
            and time.monotonic() - self._phase_started
            > self._float("alignment_timeout")
        ):
            self._abort(f"{self._state} timed out")
            return
        progress = tracking_target(
            path,
            pose[:2],
            self._cursor,
            self._float("path_lookahead"),
            self._int("maximum_segment_advance"),
        )
        self._cursor = progress.segment_index
        desired = math.atan2(
            progress.target[1] - pose[1], progress.target[0] - pose[0]
        )
        error = heading_error(pose[2], desired)
        if abs(error) <= self._float("heading_tolerance"):
            self._publish_zero()
            if require_target and not self._target_present():
                self._abort("selected target is not visible before contact")
                return
            self._set_state(next_state, f"following {next_state} route")
            return
        self._publish_command(0.0, self._bounded_angular(error))

    def _follow(
        self,
        pose: tuple[float, float, float],
        path,
        speed: float,
        tolerance: float,
        timeout: float,
        next_state: str,
        *,
        allow_rotate_in_place: bool = True,
    ) -> None:
        if (
            not self._bool("path_only_mode")
            and time.monotonic() - self._phase_started > timeout
        ):
            self._abort(f"{self._state} timed out")
            return
        if math.dist(pose[:2], path[-1]) <= tolerance:
            self._publish_zero()
            self._set_state(next_state, f"entering {next_state}")
            return
        progress = tracking_target(
            path,
            pose[:2],
            self._cursor,
            self._float("path_lookahead"),
            self._int("maximum_segment_advance"),
        )
        self._cursor = progress.segment_index
        if not self._bool("path_only_mode") and progress.deviation > self._float(
            "maximum_path_deviation"
        ):
            self._abort(f"path deviation is {progress.deviation:.3f} m")
            return
        desired = math.atan2(
            progress.target[1] - pose[1], progress.target[0] - pose[0]
        )
        error = heading_error(pose[2], desired)
        linear, angular = pure_pursuit_command(
            speed,
            self._float("minimum_linear_speed"),
            error,
            max(1e-6, math.dist(pose[:2], progress.target)),
            self._float("drive_heading_limit"),
            self._float("maximum_angular_speed"),
        )
        if not allow_rotate_in_place and linear == 0.0 and angular != 0.0:
            self._abort(f"{self._state} heading error requires in-place rotation")
            return
        self._publish_command(linear, angular)

    def _tick_release(self) -> None:
        if (
            not self._bool("path_only_mode")
            and time.monotonic() - self._phase_started
            > self._float("release_timeout")
        ):
            self._abort("release timed out")
            return
        if self._rear_obstacle():
            self._abort("obstacle detected behind the robot")
            return
        if (
            self._release_origin is None
            or self._odom_pose is None
            or self._odom_stale()
        ):
            self._abort("release odometry is unavailable")
            return
        reverse, lateral, yaw = retreat_motion(
            *self._release_origin, *self._odom_pose
        )
        if not self._bool("path_only_mode") and (
            abs(lateral) > self._float("release_lateral_limit")
            or abs(yaw) > self._float("release_heading_limit")
        ):
            self._abort("robot drifted during release")
            return
        if reverse >= self._float("release_distance"):
            self._publish_zero()
            self._set_state("align_return", "aligning with the return route")
            return
        linear = enforce_minimum_linear_speed(
            -self._float("release_speed"), self._float("minimum_linear_speed")
        )
        self._publish_command(linear, 0.0)

    def _align_home(self, pose: tuple[float, float, float]) -> None:
        assert self._task_home is not None
        if (
            not self._bool("path_only_mode")
            and time.monotonic() - self._phase_started
            > self._float("alignment_timeout")
        ):
            self._abort("home alignment timed out")
            return
        error = heading_error(pose[2], self._task_home[2])
        if abs(error) > self._float("heading_tolerance"):
            self._home_settle = 0
            self._publish_command(0.0, self._bounded_angular(error))
            return
        self._publish_zero()
        self._home_settle += 1
        if self._home_settle >= self._int("home_settle_cycles"):
            self._finish_cycle()

    def _finish_cycle(self) -> None:
        assert self._snapshot is not None
        assert self._plan is not None
        try:
            colors, counts = inventory_after_delivery(
                self._snapshot.expected_colors,
                self._snapshot.expected_color_counts,
                self._plan.selection.target.color,
            )
        except ValueError as error:
            self._abort(str(error))
            return
        if not self._services_ready():
            self._abort("perception services disappeared after the cycle")
            return

        destinations = self._delivered_destinations + [self._plan.destination]
        exclusions = SetSnapshotExclusions.Request()
        exclusions.centers = [Point(x=x, y=y, z=0.0) for x, y in destinations]
        exclusions.radius = self._float("delivered_exclusion_radius")
        inventory = SetObjectInventory.Request()
        inventory.colors = list(colors)
        inventory.counts = list(counts)
        steps = [
            ("validation reset", self._validation_reset, Trigger.Request()),
            ("snapshot reset", self._snapshot_reset, Trigger.Request()),
        ]
        if sum(counts) > 0:
            steps.extend(
                [
                    ("exclusion update", self._exclusion_update, exclusions),
                    ("inventory update", self._inventory_update, inventory),
                    ("snapshot start", self._snapshot_start, Trigger.Request()),
                ]
            )
        self._delivered_destinations = destinations
        self._expected_inventory = (tuple(colors), tuple(counts))
        self._snapshot = None
        final_state = "task_complete" if sum(counts) == 0 else "collecting_remaining"
        reason = (
            "all configured cylinders completed"
            if sum(counts) == 0
            else "waiting for the reduced inventory"
        )
        self._state = "finalizing"
        self._publish_status()
        self._run_steps(steps, final_state, reason)

    def _run_steps(self, steps, final_state: str, final_reason: str) -> None:
        self._generation += 1
        generation = self._generation
        self._operation_busy = True

        def run(index: int) -> None:
            if generation != self._generation:
                return
            if index == len(steps):
                self._operation_busy = False
                self._state = final_state
                self._reason = final_reason
                self._plan = None
                self._phase_started = time.monotonic()
                if final_state == "task_complete":
                    self._run_all_active = False
                self._publish_status()
                return
            label, client, request = steps[index]
            if label == "snapshot start":
                self._collection_started = self._now_ros()
                self._snapshot = None
            future = client.call_async(request)

            def completed(done) -> None:
                if generation != self._generation:
                    return
                try:
                    result = done.result()
                except Exception as error:
                    self._operation_busy = False
                    self._abort(f"{label} failed: {error}")
                    return
                if not result.success:
                    self._operation_busy = False
                    self._abort(f"{label} rejected: {result.message}")
                    return
                run(index + 1)

            future.add_done_callback(completed)

        run(0)

    def _set_state(self, state: str, reason: str) -> None:
        self._state = state
        self._reason = reason
        self._phase_started = time.monotonic()
        self._cursor = 0
        if state == "align_home":
            self._home_settle = 0
        self._publish_status()

    def _map_pose_jumped(self, pose: tuple[float, float, float]) -> bool:
        now = time.monotonic()
        previous = self._previous_map_pose
        previous_time = self._previous_map_pose_time
        self._previous_map_pose = pose
        self._previous_map_pose_time = now
        if previous is None or previous_time <= 0.0:
            return False
        elapsed = max(0.0, now - previous_time)
        position_limit = self._float("maximum_map_position_step") + 0.25 * elapsed
        yaw_limit = self._float("maximum_map_yaw_step") + 0.75 * elapsed
        return math.dist(pose[:2], previous[:2]) > position_limit or abs(
            heading_error(previous[2], pose[2])
        ) > yaw_limit

    def _robot_pose(self, frame: str) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                frame,
                self._string("robot_frame"),
                Time(),
                timeout=Duration(seconds=self._float("transform_timeout")),
            ).transform
        except TransformException:
            return None
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        return transform.translation.x, transform.translation.y, yaw

    def _planar_transform(self, target: str, source: str):
        if target == source:
            return (0.0, 0.0), 0.0
        try:
            transform = self._tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=self._float("transform_timeout")),
            ).transform
        except TransformException as error:
            raise ValueError(f"transform {source} to {target} is unavailable") from error
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        return (transform.translation.x, transform.translation.y), yaw

    def _scan_stale(self, timeout: float) -> bool:
        if self._scan is None or self._scan_received <= 0.0:
            return True
        receive_age = time.monotonic() - self._scan_received
        stamp_age = self._now_ros() - self._scan_stamp
        return (
            receive_age > timeout
            or stamp_age < -0.1
            or stamp_age > self._float("message_stamp_timeout")
        )

    def _odom_stale(self) -> bool:
        if self._odom_pose is None or self._odom_received <= 0.0:
            return True
        receive_age = time.monotonic() - self._odom_received
        stamp_age = self._now_ros() - self._odom_stamp
        return (
            receive_age > self._float("odom_timeout")
            or stamp_age < -0.1
            or stamp_age > self._float("message_stamp_timeout")
        )

    def _exclusive_command_owner(self) -> bool:
        infos = self.get_publishers_info_by_topic(self._string("cmd_vel_topic"))
        return len(infos) == 1

    def _target_present(self) -> bool:
        if self._scan is None:
            return False
        scan = self._scan
        return front_target_present(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._float("target_acquire_distance"),
            self._float("target_corridor_half_width"),
        )

    def _unexpected_front_obstacle(self, target_depth: float) -> bool:
        if not self._bool("live_obstacle_stop_enabled"):
            return False
        if self._scan is None:
            return True
        scan = self._scan
        return unexpected_obstacle_ahead(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._float("obstacle_stop_distance"),
            self._float("obstacle_half_width"),
            self._float("target_corridor_half_width"),
            target_depth,
        )

    def _front_obstacle(self) -> bool:
        return self._unexpected_front_obstacle(1e-6)

    def _rear_obstacle(self) -> bool:
        if not self._bool("live_obstacle_stop_enabled"):
            return False
        if self._scan is None:
            return True
        scan = self._scan
        return obstacle_behind(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self._float("rear_stop_distance"),
            self._float("obstacle_half_width"),
        )

    def _publish_plan(self, plan: DirectCyclePlan) -> None:
        stamp = self.get_clock().now().to_msg()
        frame = self._string("fixed_frame")
        complete = self._deduplicate(
            (*plan.approach_path, *plan.contact_path, *plan.push_path, *plan.return_path)
        )
        message = Path()
        message.header.frame_id = frame
        message.header.stamp = stamp
        for x, y in complete:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self._path_pub.publish(message)

        clear = Marker(action=Marker.DELETEALL)
        target = Marker()
        target.header = message.header
        target.ns = "selected_target"
        target.id = int(plan.selection.target.candidate_id)
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = plan.selection.target.x
        target.pose.position.y = plan.selection.target.y
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.12
        target.color.r = 1.0
        target.color.a = 1.0
        self._marker_pub.publish(MarkerArray(markers=[clear, target]))

    @staticmethod
    def _deduplicate(points):
        output = []
        for point in points:
            if not output or math.dist(output[-1], point) > 1e-9:
                output.append(point)
        return tuple(output)

    def _publish_command(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = linear
        command.linear.y = 0.0
        command.angular.z = angular
        self._cmd_pub.publish(command)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())

    def _terminate(self, state: str, reason: str, *, clear_plan: bool = False) -> None:
        self._state = state
        self._reason = reason
        self._stop_burst = 5
        self._publish_zero()
        if clear_plan:
            self._snapshot = None
            self._plan = None
        self._publish_status()

    def _abort(self, reason: str) -> None:
        self.get_logger().error(reason)
        self._generation += 1
        self._operation_busy = False
        self._run_all_active = False
        self._terminate("failed", reason)

    def _publish_status(self) -> None:
        plan = self._plan
        payload = {
            "architecture": "simple_map_route",
            "execution_enabled": self._bool("execution_enabled"),
            "path_only_mode": self._bool("path_only_mode"),
            "fixed_localization_ready": self._fixed_localization_ready,
            "localization_owner": self._localization_owner,
            "state": self._state,
            "reason": self._reason,
            "run_all_active": self._run_all_active,
            "target_id": plan.selection.target.candidate_id if plan else -1,
            "target_color": plan.selection.target.color if plan else "",
            "delivered_count": len(self._delivered_destinations),
            "remaining_counts": (
                list(self._expected_inventory[1])
                if self._expected_inventory is not None
                else []
            ),
        }
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _services_ready(self) -> bool:
        perception_ready = all(
            client.service_is_ready()
            for client in (
                self._validation_reset,
                self._snapshot_reset,
                self._snapshot_start,
                self._inventory_update,
                self._exclusion_update,
            )
        )
        return perception_ready

    def _validate_parameters(self) -> None:
        if self._string("fixed_frame") != "map":
            raise ValueError("simple_task3 requires fixed_frame=map")
        if self._string("cmd_vel_topic") != "/cmd_vel":
            raise ValueError("cmd_vel_topic must be /cmd_vel")
        positive = (
            "control_rate",
            "minimum_linear_speed",
            "approach_speed",
            "contact_speed",
            "push_speed",
            "release_speed",
            "return_speed",
            "maximum_angular_speed",
            "path_lookahead",
            "maximum_path_deviation",
            "scan_timeout",
            "odom_timeout",
        )
        if any(
            not math.isfinite(self._float(name)) or self._float(name) <= 0
            for name in positive
        ):
            raise ValueError("motion rates, limits and timeouts must be positive")

    @staticmethod
    def _stamp_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _now_ros(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _bounded_angular(self, error: float) -> float:
        limit = self._float("maximum_angular_speed")
        return max(-limit, min(limit, self._float("angular_gain") * error))

    def _failure(self, response, reason: str):
        response.success = False
        response.message = reason
        return response

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def prepare_shutdown(self) -> None:
        self._generation += 1
        self._run_all_active = False
        for _unused in range(5):
            self._publish_zero()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimpleTaskController()
    executor = MultiThreadedExecutor(num_threads=3)
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
