"""Compact Task 3 controller using straight, non-replanning motion segments."""

from __future__ import annotations

from dataclasses import replace
import json
import math
import signal
from collections import deque
from statistics import median
import threading
import time

import rclpy
from color_object_sorter_interfaces.msg import ValidatedCylinderArray
from color_object_sorter_interfaces.srv import (
    SetObjectInventory,
    SetSnapshotExclusions,
)
from cylinder_field_mapping.lidar_candidates import extract_candidates
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
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .direct_cycle import (
    DirectCyclePlan,
    build_direct_cycle,
    enforce_minimum_linear_speed,
    heading_error,
    polyline_tracking_target,
    pure_pursuit_command,
    stitch_reacquired_return_path,
    transform_point_2d,
)
from .execution_safety import (
    front_target_present,
    match_reacquisition_reference,
    obstacle_behind,
    odometry_step_is_plausible,
    retreat_motion,
    unexpected_obstacle_ahead,
)
from .geometry import Target, destination_slot, local_offset_to_map, select_right_first
from .inventory_cycle import inventory_after_delivery
from .inertial_heading import (
    InertialHeadingTracker,
    heading_is_settled,
    stable_circular_heading,
)
from .task_workflow import (
    automatic_cycle_ready,
    snapshot_matches_collection,
    validate_inventory,
)


ACTIVE_STATES = {
    "aligning_approach",
    "approaching",
    "aligning_contact",
    "reacquiring_target",
    "contacting",
    "pushing",
    "releasing",
    "aligning_return",
    "returning",
    "aligning_home",
}


class DirectTaskControllerNode(Node):
    """Plan and execute one complete push cycle in a single ROS node.

    This controller deliberately does not call Nav2 or dynamically replan. It
    follows paths generated outside the locked remaining-cylinder keepout.
    Perception remains in the proven detector/snapshot/validator nodes.
    """

    def __init__(self) -> None:
        super().__init__("cylinder_direct_task_controller")
        defaults = {
            "execution_enabled": False,
            "snapshot_topic": "/cylinder_snapshot/validated_objects",
            "scan_topic": "/scan",
            "cmd_vel_topic": "/cmd_vel",
            # Keep perceived targets, keepouts, generated paths, and the live
            # tracking pose in one fixed frame.  Freezing map geometry into
            # odom lets later SLAM corrections rotate a safe path through the
            # very cylinders it was planned to avoid.
            "fixed_frame": "map",
            "execution_frame": "map",
            "robot_frame": "base_footprint",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "inertial_heading_enabled": True,
            "imu_topic": "/imu/data_raw",
            "imu_timeout": 0.50,
            "imu_maximum_integration_gap": 0.30,
            "imu_bias_sample_count": 30,
            "imu_stationary_rate_limit": 0.05,
            "absolute_home_heading_enabled": True,
            "home_heading_sample_count": 5,
            "home_heading_sample_spread": 0.035,
            "inventory_colors": ["blue", "green", "red"],
            "inventory_counts": [2, 2, 2],
            "destination_colors": ["blue", "green", "red"],
            "staging_clearance": 0.25,
            "contact_offset": 0.15,
            "release_distance": 0.15,
            "destination_slot_spacing": 0.18,
            "envelope_exit_clearance": 0.10,
            "delivered_exclusion_radius": 0.18,
            "transit_clearance": 0.10,
            "require_clear_corridors": True,
            "live_obstacle_stop_enabled": True,
            "scan_timeout": 0.75,
            "motion_scan_timeout": 2.0,
            "transform_timeout": 0.10,
            "transform_failure_limit": 3,
            "control_rate": 10.0,
            "minimum_linear_speed": 0.15,
            "transit_speed": 0.15,
            "contact_speed": 0.15,
            "push_speed": 0.15,
            "release_speed": 0.15,
            "return_speed": 0.15,
            "angular_gain": 3.0,
            "maximum_angular_speed": 0.50,
            "heading_tolerance": 0.08,
            "heading_settle_rate": 0.05,
            "heading_settle_required_cycles": 3,
            "drive_heading_limit": 0.60,
            "path_lookahead": 0.15,
            "staging_tolerance": 0.05,
            "contact_tolerance": 0.035,
            "push_tolerance": 0.06,
            "home_tolerance": 0.08,
            "maximum_path_deviation": 0.12,
            "return_path_deviation_guard_enabled": False,
            "odometry_timeout": 0.75,
            "odometry_jump_guard_enabled": True,
            "maximum_odom_linear_speed": 0.50,
            "odom_distance_slack": 0.03,
            "maximum_home_anchor_error_before_plan": 0.30,
            "alignment_timeout": 15.0,
            "approach_timeout": 40.0,
            "contact_timeout": 15.0,
            "push_timeout": 60.0,
            "release_timeout": 10.0,
            "release_forward_motion_guard_enabled": False,
            "return_timeout": 50.0,
            "snapshot_wait_timeout": 120.0,
            "target_acquire_distance": 0.65,
            "target_maximum_distance": 0.32,
            "target_corridor_half_width": 0.07,
            "target_reacquisition_enabled": True,
            "target_reacquisition_timeout": 2.0,
            "target_reacquisition_required_scans": 3,
            "target_reacquisition_maximum_correction": 0.20,
            "target_reacquisition_ambiguity_margin": 0.06,
            "target_reacquisition_sample_spread": 0.04,
            "target_reacquisition_maximum_point_gap": 0.05,
            "target_reacquisition_minimum_points": 3,
            "target_reacquisition_minimum_diameter": 0.015,
            "target_reacquisition_maximum_diameter": 0.12,
            "target_nominal_radius": 0.035,
            "obstacle_stop_distance": 0.32,
            "obstacle_half_width": 0.18,
            "rear_stop_distance": 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        destination_defaults = {
            "blue": (0.00, 2.00),
            "green": (2.00, 0.00),
            "red": (0.00, -2.00),
        }
        for color in self.get_parameter("destination_colors").value:
            default = destination_defaults.get(str(color))
            for axis, index in (("x", 0), ("y", 1)):
                name = f"destinations.{color}.{axis}"
                if default is None:
                    self.declare_parameter(name, Parameter.Type.DOUBLE)
                else:
                    self.declare_parameter(name, default[index])

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            String, "/cylinder_task/status", latched
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, "/cylinder_push_plan/markers", latched
        )
        self._approach_publisher = self.create_publisher(
            Path, "/cylinder_push_plan/approach_path", latched
        )
        self._contact_publisher = self.create_publisher(
            Path, "/cylinder_push_plan/contact_path", latched
        )
        self._push_publisher = self.create_publisher(
            Path, "/cylinder_push_plan/robot_push_path", latched
        )
        self._return_publisher = self.create_publisher(
            Path, "/cylinder_push_plan/return_path", latched
        )
        self._cmd_publisher = self.create_publisher(
            Twist, self._string("cmd_vel_topic"), 10
        )
        # Convex-keepout path generation runs synchronously in the controller
        # timer. Keep sensor callbacks in a different callback group so the
        # multi-threaded executor can continue recording fresh scan and odom
        # samples while that CPU-bound planning callback is active.
        self._sensor_callback_group = MutuallyExclusiveCallbackGroup()
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
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            Odometry,
            self._string("odom_topic"),
            self._on_odom,
            20,
            callback_group=self._sensor_callback_group,
        )
        self.create_subscription(
            Imu,
            self._string("imu_topic"),
            self._on_imu,
            qos_profile_sensor_data,
            callback_group=self._sensor_callback_group,
        )
        self.create_service(Trigger, "/cylinder_task/prepare", self._prepare)
        self.create_service(Trigger, "/cylinder_task/run_once", self._run_once)
        self.create_service(Trigger, "/cylinder_task/run_all", self._run_all)
        self.create_service(Trigger, "/cylinder_task/cancel", self._cancel)

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
        self._reason = "call /cylinder_task/prepare for a new field"
        self._generation = 0
        self._snapshot: ValidatedCylinderArray | None = None
        self._scan: LaserScan | None = None
        self._last_scan_received = 0.0
        self._plan: DirectCyclePlan | None = None
        self._plan_targets: tuple[Target, ...] = ()
        self._target_reacquired = False
        self._reacquisition_reference_frame = ""
        self._reacquisition_reference_point: tuple[float, float] | None = None
        self._reacquisition_samples: list[tuple[float, float]] = []
        self._reacquisition_scan_stamp: tuple[int, int] | None = None
        self._reacquisition_reason = "waiting for a fresh scan"
        self._task_home: tuple[float, float, float] | None = None
        self._task_home_fixed_yaw: float | None = None
        self._home_heading_samples = deque(
            maxlen=max(1, self._int("home_heading_sample_count"))
        )
        self._current_fixed_yaw: float | None = None
        self._current_map_to_odom_yaw: float | None = None
        self._home_heading_tf_failures = 0
        self._field_center: tuple[float, float] | None = None
        self._initial_counts: dict[str, int] = {}
        self._expected_inventory: tuple[tuple[str, ...], tuple[int, ...]] | None = None
        self._delivered_destinations: list[tuple[float, float]] = []
        self._phase_started = 0.0
        self._odom_sample: tuple[float, float, float] | None = None
        self._previous_odom_sample: tuple[float, float, float] | None = None
        self._last_odom_received = 0.0
        self._inertial_lock = threading.Lock()
        self._inertial_heading = InertialHeadingTracker(
            self._float("imu_maximum_integration_gap")
        )
        self._latest_imu_sample: tuple[float, float] | None = None
        self._last_imu_received = 0.0
        self._imu_gap_invalid = False
        self._imu_bias_samples = deque(
            maxlen=max(1, self._int("imu_bias_sample_count"))
        )
        self._heading_settle_cycles = 0
        self._release_origin: tuple[float, float, float] | None = None
        self._tf_failures = 0
        self._stop_burst = 0
        self._operation_busy = False
        self._run_all_active = False
        self._collection_started = 0.0
        self.create_timer(1.0 / self._float("control_rate"), self._tick)
        self._publish_status()
        self.get_logger().info(
            "Compact direct Task 3 controller ready; Nav2 is not used"
        )

    def _on_snapshot(self, message: ValidatedCylinderArray) -> None:
        if self._state not in ("collecting", "collecting_remaining"):
            return
        if self._expected_inventory is None:
            return
        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        colors, counts = self._expected_inventory
        if not snapshot_matches_collection(
            colors,
            counts,
            message.expected_colors,
            message.expected_color_counts,
            stamp,
            self._collection_started,
        ):
            return
        self._snapshot = message

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = message
        self._last_scan_received = self._now()

    def _on_odom(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self._string("odom_frame")
            or message.child_frame_id != self._string("robot_frame")
        ):
            return
        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        self._odom_sample = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            stamp,
        )
        self._last_odom_received = self._now()

    def _on_imu(self, message: Imu) -> None:
        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        angular_rate = float(message.angular_velocity.z)
        if not math.isfinite(stamp) or not math.isfinite(angular_rate):
            return
        received = self._now()
        with self._inertial_lock:
            self._latest_imu_sample = (stamp, angular_rate)
            self._last_imu_received = received
            if (
                self._state not in ACTIVE_STATES
                and abs(angular_rate) <= self._float("imu_stationary_rate_limit")
            ):
                self._imu_bias_samples.append(angular_rate)
            if self._inertial_heading.initialized:
                integrated = self._inertial_heading.observe(
                    stamp,
                    angular_rate,
                    integrate=self._state in ACTIVE_STATES,
                )
                if not integrated:
                    self._imu_gap_invalid = True

    def _prepare(self, request, response):
        del request
        return self._begin_preparation(response, run_all=False)

    def _run_all(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._failure(response, "execution_enabled is false")
        return self._begin_preparation(response, run_all=True)

    def _begin_preparation(self, response, *, run_all: bool):
        if (
            self._state in ACTIVE_STATES
            or self._operation_busy
            or self._run_all_active
        ):
            return self._failure(response, f"controller is busy: {self._state}")
        colors = [str(value) for value in self.get_parameter("inventory_colors").value]
        counts = [int(value) for value in self.get_parameter("inventory_counts").value]
        try:
            validate_inventory(colors, counts)
        except ValueError as error:
            return self._failure(response, str(error))
        if not self._services_ready():
            return self._failure(response, "perception reset services are unavailable")

        self._run_all_active = run_all
        reason = (
            "resetting and starting an automatic full-inventory task"
            if run_all
            else "resetting one compact task"
        )
        self._terminate("preparing", reason, clear_plan=True)
        self._task_home = None
        self._task_home_fixed_yaw = None
        self._home_heading_samples.clear()
        self._current_fixed_yaw = None
        self._current_map_to_odom_yaw = None
        self._home_heading_tf_failures = 0
        with self._inertial_lock:
            self._inertial_heading.clear()
            self._imu_gap_invalid = False
        self._heading_settle_cycles = 0
        self._field_center = None
        self._initial_counts = dict(zip(colors, counts))
        self._expected_inventory = (tuple(colors), tuple(counts))
        self._delivered_destinations.clear()
        exclusions = SetSnapshotExclusions.Request()
        exclusions.centers = []
        exclusions.radius = 0.0
        inventory = SetObjectInventory.Request()
        inventory.colors = colors
        inventory.counts = counts
        steps = [
            ("validation reset", self._validation_reset, Trigger.Request()),
            ("snapshot reset", self._snapshot_reset, Trigger.Request()),
            ("exclusions clear", self._exclusion_update, exclusions),
            ("inventory update", self._inventory_update, inventory),
            ("snapshot start", self._snapshot_start, Trigger.Request()),
        ]
        self._run_service_steps(
            steps,
            "collecting",
            "collecting the configured cylinder inventory",
        )
        response.success = True
        response.message = (
            "automatic full-inventory task accepted; collecting the first snapshot"
            if run_all
            else "preparation accepted; wait for a locked validated snapshot"
        )
        return response

    def _run_once(self, request, response):
        del request
        if not self._bool("execution_enabled"):
            return self._failure(response, "execution_enabled is false")
        if self._run_all_active:
            return self._failure(response, "automatic full-inventory task is active")
        if self._state in ACTIVE_STATES or self._operation_busy:
            return self._failure(response, f"controller is busy: {self._state}")
        try:
            message = self._start_cycle()
        except ValueError as error:
            return self._failure(response, str(error))
        response.success = True
        response.message = message
        return response

    def _start_cycle(self) -> str:
        if (
            self._snapshot is None
            or not self._snapshot.ready
            or not self._snapshot.locked
        ):
            raise ValueError("validated snapshot is not ready and locked")
        pose = self._robot_pose()
        if pose is None:
            raise ValueError("robot transform is unavailable")
        if self._scan_stale():
            raise ValueError("laser scan is missing or stale")
        if self._odometry_stale():
            raise ValueError("odometry is missing or stale")
        if self._bool("inertial_heading_enabled"):
            self._start_or_resume_inertial_heading(pose[2])
        fixed_home_yaw = self._task_home_fixed_yaw
        if (
            fixed_home_yaw is None
            and self._bool("absolute_home_heading_enabled")
        ):
            fixed_home_yaw = self._robot_yaw_in_frame(
                self._string("fixed_frame")
            )
            if fixed_home_yaw is None:
                raise ValueError("fixed-frame home heading is unavailable")
        plan = self._make_plan(pose, self._snapshot)
        reference_frame, reference_point = self._source_target_reference(
            self._snapshot,
            plan.selection.target.candidate_id,
        )
        self._plan = plan
        if self._task_home_fixed_yaw is None:
            self._task_home_fixed_yaw = fixed_home_yaw
        self._reacquisition_reference_frame = reference_frame
        self._reacquisition_reference_point = reference_point
        self._target_reacquired = False
        self._reacquisition_samples.clear()
        self._reacquisition_scan_stamp = None
        self._publish_plan(plan)
        self._generation += 1
        self._previous_odom_sample = self._odom_sample
        self._tf_failures = 0
        self._start_phase("aligning_approach", pose[:2])
        return (
            f"direct cycle started for candidate {plan.selection.target.candidate_id} "
            f"({plan.selection.target.color})"
        )

    def _cancel(self, request, response):
        del request
        if (
            self._state not in ACTIVE_STATES
            and not self._operation_busy
            and not self._run_all_active
        ):
            return self._failure(response, "no active motion or reset operation")
        self._generation += 1
        self._operation_busy = False
        self._run_all_active = False
        self._terminate("cancelled", "operator requested cancellation")
        response.success = True
        response.message = "cancelled; zero velocity requested"
        return response

    def _make_plan(
        self,
        pose: tuple[float, float, float],
        snapshot: ValidatedCylinderArray,
    ) -> DirectCyclePlan:
        source_frame = snapshot.header.frame_id or self._string("fixed_frame")
        execution_frame = self._string("execution_frame")
        translation, rotation = self._planar_transform(
            execution_frame,
            source_frame,
        )
        targets = tuple(
            Target(
                candidate_id=item.candidate_id,
                x=transformed[0],
                y=transformed[1],
                color=item.color,
                radius=item.radius,
            )
            for item in snapshot.objects
            if item.state == "validated"
            for transformed in (
                transform_point_2d(
                    (item.position.x, item.position.y),
                    translation,
                    rotation,
                ),
            )
        )
        expected = sum(int(value) for value in snapshot.expected_color_counts)
        if len(targets) != expected:
            raise ValueError(
                f"locked snapshot contains {len(targets)} validated targets, "
                f"expected {expected}"
            )
        selection = select_right_first(
            targets, pose[0], pose[1], self._float("staging_clearance")
        )
        task_home = self._task_home if self._task_home is not None else pose
        if self._task_home is not None and math.dist(
            pose[:2], self._task_home[:2]
        ) > self._float("maximum_home_anchor_error_before_plan"):
            raise ValueError(
                "robot is not near the saved task home; call prepare after "
                "restarting mapping"
            )
        field_center = (
            self._field_center
            if self._field_center is not None
            else (selection.envelope_x, selection.envelope_y)
        )
        initial_counts = self._initial_counts
        if not initial_counts:
            initial_counts = dict(
                zip(
                    (str(value) for value in snapshot.expected_colors),
                    (int(value) for value in snapshot.expected_color_counts),
                )
            )
        destination = self._destination(
            selection.target.color,
            snapshot,
            field_center,
            task_home,
            initial_counts,
        )
        distance_from_center = math.dist(destination, field_center)
        if distance_from_center < (
            selection.envelope_radius + self._float("envelope_exit_clearance")
        ):
            raise ValueError(
                "destination must be outside the initial cylinder envelope"
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
            home=task_home,
            field_center=field_center,
            require_clear_corridors=self._bool("require_clear_corridors"),
        )
        # Anchors are transactional: a rejected preview must not retain a
        # partial task frame that can survive a later SLAM restart.
        self._task_home = task_home
        self._field_center = field_center
        self._initial_counts = dict(initial_counts)
        self._plan_targets = targets
        return plan

    def _source_target_reference(
        self,
        snapshot: ValidatedCylinderArray,
        candidate_id: int,
    ) -> tuple[str, tuple[float, float]]:
        """Return the selected target in its original perception frame."""

        matches = tuple(
            item
            for item in snapshot.objects
            if item.state == "validated" and item.candidate_id == candidate_id
        )
        if len(matches) != 1:
            raise ValueError(
                f"selected candidate {candidate_id} has {len(matches)} "
                "fixed-frame references"
            )
        item = matches[0]
        frame = snapshot.header.frame_id or self._string("fixed_frame")
        return frame, (float(item.position.x), float(item.position.y))

    def _destination(
        self,
        color: str,
        snapshot: ValidatedCylinderArray,
        field_center: tuple[float, float],
        task_home: tuple[float, float, float],
        initial_counts: dict[str, int],
    ) -> tuple[float, float]:
        colors = tuple(
            str(value)
            for value in self.get_parameter("destination_colors").value
        )
        if color not in colors:
            raise ValueError(f"no destination configured for color {color!r}")
        current = dict(
            zip(
                (str(value) for value in snapshot.expected_colors),
                (int(value) for value in snapshot.expected_color_counts),
            )
        )
        if current.get(color, 0) < 1:
            raise ValueError(f"selected color {color!r} is absent from inventory")
        initial = initial_counts.get(color, current[color])
        slot_index = initial - current[color]
        local_x, local_y = destination_slot(
            self._float(f"destinations.{color}.x"),
            self._float(f"destinations.{color}.y"),
            slot_index,
            initial,
            self._float("destination_slot_spacing"),
        )
        return local_offset_to_map(
            field_center[0],
            field_center[1],
            task_home[2],
            local_x,
            local_y,
        )

    def _tick(self) -> None:
        if self._stop_burst > 0:
            self._publish_zero()
            self._stop_burst -= 1
        if self._state not in ACTIVE_STATES:
            self._tick_run_all_wait()
            return
        if self._scan_stale(self._float("motion_scan_timeout")):
            self._abort(
                f"laser scan became stale during {self._state}; "
                f"age={self._scan_age():.3f}s"
            )
            return
        if self._bool("inertial_heading_enabled"):
            if self._imu_stale():
                self._abort("IMU angular-rate input became stale")
                return
            with self._inertial_lock:
                invalid_gap = self._imu_gap_invalid
                self._imu_gap_invalid = False
            if invalid_gap:
                self._abort("IMU angular-rate integration gap exceeded its limit")
                return
        pose = self._robot_pose()
        if pose is None:
            self._tf_failures += 1
            if self._tf_failures >= self._int("transform_failure_limit"):
                self._abort("robot transform was lost")
            return
        self._tf_failures = 0
        if self._odometry_stale() or self._odom_sample is None:
            self._abort("odometry became stale")
            return
        current_odom = self._odom_sample
        previous_odom = self._previous_odom_sample
        if previous_odom is not None and current_odom[2] < previous_odom[2]:
            self._abort("odometry timestamp moved backwards")
            return
        if previous_odom is not None and current_odom[2] > previous_odom[2]:
            elapsed = current_odom[2] - previous_odom[2]
            if self._bool("odometry_jump_guard_enabled"):
                try:
                    plausible = odometry_step_is_plausible(
                        previous_odom[0],
                        previous_odom[1],
                        current_odom[0],
                        current_odom[1],
                        elapsed,
                        self._float("maximum_odom_linear_speed"),
                        self._float("odom_distance_slack"),
                    )
                except ValueError as error:
                    self._abort(str(error))
                    return
                if not plausible:
                    step = math.hypot(
                        current_odom[0] - previous_odom[0],
                        current_odom[1] - previous_odom[1],
                    )
                    self._abort(
                        f"odometry pose changed by {step:.3f} m in "
                        f"{elapsed:.3f} s"
                    )
                    return
            self._previous_odom_sample = current_odom
        assert self._plan is not None

        if self._state == "aligning_approach":
            target, _ = polyline_tracking_target(
                self._plan.approach_path,
                pose[:2],
                self._float("path_lookahead"),
            )
            self._tick_alignment(pose, target, "approaching")
        elif self._state == "approaching":
            if self._live_obstacle_stop_enabled() and self._front_obstacle():
                self._abort("obstacle entered the direct approach corridor")
            else:
                self._tick_drive(
                    pose,
                    self._plan.approach_path,
                    self._float("transit_speed"),
                    self._float("staging_tolerance"),
                    self._float("approach_timeout"),
                    "aligning_contact",
                )
        elif self._state == "aligning_contact":
            self._tick_alignment(pose, self._plan.contact, "contacting")
        elif self._state == "reacquiring_target":
            self._tick_target_reacquisition(pose)
        elif self._state == "contacting":
            self._tick_contact(pose)
        elif self._state == "pushing":
            self._tick_push(pose)
        elif self._state == "releasing":
            self._tick_release(pose)
        elif self._state == "aligning_return":
            target, _ = polyline_tracking_target(
                self._plan.return_path,
                pose[:2],
                self._float("path_lookahead"),
            )
            self._tick_alignment(pose, target, "returning")
        elif self._state == "returning":
            if self._live_obstacle_stop_enabled() and self._front_obstacle():
                self._abort("obstacle entered the direct return corridor")
            else:
                self._tick_return(pose)
        elif self._state == "aligning_home":
            home_pose = pose
            heading_stable = True
            if self._bool("absolute_home_heading_enabled"):
                if self._task_home_fixed_yaw is None:
                    self._abort("fixed-frame home heading is unavailable")
                    return
                current_yaw = self._robot_yaw_in_frame(
                    self._string("fixed_frame")
                )
                if current_yaw is None:
                    self._home_heading_tf_failures += 1
                    if self._home_heading_tf_failures >= self._int(
                        "transform_failure_limit"
                    ):
                        self._abort("fixed-frame home heading transform was lost")
                    return
                self._home_heading_tf_failures = 0
                self._home_heading_samples.append(current_yaw)
                filtered_yaw, heading_stable = stable_circular_heading(
                    tuple(self._home_heading_samples),
                    self._int("home_heading_sample_count"),
                    self._float("home_heading_sample_spread"),
                )
                self._current_fixed_yaw = (
                    current_yaw if filtered_yaw is None else filtered_yaw
                )
                try:
                    _, self._current_map_to_odom_yaw = self._planar_transform(
                        self._string("fixed_frame"),
                        self._string("odom_frame"),
                    )
                except ValueError:
                    self._current_map_to_odom_yaw = None
                home_pose = (pose[0], pose[1], self._current_fixed_yaw)
                desired_yaw = self._task_home_fixed_yaw
            elif self._bool("inertial_heading_enabled"):
                corrected_yaw = self._inertial_yaw()
                if corrected_yaw is None:
                    self._abort("inertial heading is unavailable")
                    return
                # Use the integrated heading only for final in-place alignment;
                # mixing it with TF x/y during path tracking would create an
                # inconsistent coordinate system.
                home_pose = (pose[0], pose[1], corrected_yaw)
                desired_yaw = self._plan.home[2]
            else:
                desired_yaw = self._plan.home[2]
            self._tick_yaw(
                home_pose,
                desired_yaw,
                heading_stable=heading_stable,
            )

    def _tick_run_all_wait(self) -> None:
        if not self._run_all_active:
            return
        snapshot = self._snapshot
        snapshot_ready = snapshot is not None and snapshot.ready
        snapshot_locked = snapshot is not None and snapshot.locked
        if self._operation_busy or self._state not in (
            "collecting",
            "collecting_remaining",
        ):
            return
        if self._now() - self._collection_started > self._float(
            "snapshot_wait_timeout"
        ):
            self._abort(f"timed out waiting for a locked snapshot: {self._state}")
            return
        if not automatic_cycle_ready(
            self._state,
            snapshot_ready,
            snapshot_locked,
            self._operation_busy,
        ):
            return
        try:
            message = self._start_cycle()
        except ValueError as error:
            self._abort(f"automatic cycle start failed: {error}")
            return
        self.get_logger().info(message)

    def _tick_alignment(
        self,
        pose: tuple[float, float, float],
        target: tuple[float, float],
        next_state: str,
    ) -> None:
        if self._now() - self._phase_started > self._float("alignment_timeout"):
            self._abort(f"{self._state} timed out")
            return
        desired = math.atan2(target[1] - pose[1], target[0] - pose[0])
        error = heading_error(pose[2], desired)
        if abs(error) <= self._float("heading_tolerance"):
            self._publish_zero()
            if next_state == "contacting":
                if (
                    self._bool("target_reacquisition_enabled")
                    and not self._target_reacquired
                ):
                    self._start_phase("reacquiring_target", pose[:2])
                    return
                if not self._target_present(self._float("target_acquire_distance")):
                    self._abort(
                        "selected target is not visible in the contact corridor"
                    )
                    return
            self._start_phase(next_state, pose[:2])
            return
        self._publish_command(0.0, self._bounded_angular(error))

    def _tick_target_reacquisition(
        self, pose: tuple[float, float, float]
    ) -> None:
        """Correct the selected target from several unique near-range scans."""

        self._publish_zero()
        if self._now() - self._phase_started > self._float(
            "target_reacquisition_timeout"
        ):
            self._abort("target reacquisition timed out: " + self._reacquisition_reason)
            return
        scan = self._scan
        if scan is None:
            self._reacquisition_reason = "laser scan is unavailable"
            return
        stamp = (
            int(scan.header.stamp.sec),
            int(scan.header.stamp.nanosec),
        )
        if stamp == self._reacquisition_scan_stamp:
            return
        self._reacquisition_scan_stamp = stamp
        assert self._plan is not None
        scan_frame = scan.header.frame_id
        if not scan_frame:
            self._reacquisition_reason = "laser scan frame is empty"
            return
        reference = self._reacquisition_reference_point
        reference_frame = self._reacquisition_reference_frame
        if reference is None or not reference_frame:
            self._abort("selected target fixed-frame reference is unavailable")
            return
        scan_time = Time.from_msg(scan.header.stamp)
        try:
            scan_from_reference = self._planar_transform(
                scan_frame,
                reference_frame,
                at_time=scan_time,
            )
            execution_from_scan = self._planar_transform(
                self._string("execution_frame"),
                scan_frame,
                at_time=scan_time,
            )
            execution_from_reference = self._planar_transform(
                self._string("execution_frame"),
                reference_frame,
                at_time=scan_time,
            )
        except ValueError as error:
            self._reacquisition_reason = str(error)
            return
        try:
            extracted = extract_candidates(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                scan.range_min,
                scan.range_max,
                maximum_point_gap=self._float(
                    "target_reacquisition_maximum_point_gap"
                ),
                minimum_points=self._int("target_reacquisition_minimum_points"),
                minimum_diameter=self._float(
                    "target_reacquisition_minimum_diameter"
                ),
                maximum_diameter=self._float(
                    "target_reacquisition_maximum_diameter"
                ),
                nominal_radius=self._float("target_nominal_radius"),
            )
            points = tuple(
                (candidate.x, candidate.y)
                for candidate in extracted
                if candidate.x > 0.0
                and math.hypot(candidate.x, candidate.y)
                <= self._float("target_acquire_distance")
            )
            match, expected, reason = match_reacquisition_reference(
                points,
                reference,
                *scan_from_reference,
                self._float("target_reacquisition_maximum_correction"),
                self._float("target_reacquisition_ambiguity_margin"),
            )
        except ValueError as error:
            self._abort(f"invalid target reacquisition configuration: {error}")
            return
        self._reacquisition_reason = reason
        if match is None:
            return
        association_error = math.dist(match, expected)
        measured = transform_point_2d(match, *execution_from_scan)
        if self._reacquisition_samples:
            center = (
                median(item[0] for item in self._reacquisition_samples),
                median(item[1] for item in self._reacquisition_samples),
            )
            if math.dist(measured, center) > self._float(
                "target_reacquisition_sample_spread"
            ):
                self._reacquisition_samples.clear()
                self._reacquisition_reason = "near-range target samples are unstable"
        self._reacquisition_samples.append(measured)
        required = self._int("target_reacquisition_required_scans")
        if required < 1:
            self._abort("target_reacquisition_required_scans must be positive")
            return
        if len(self._reacquisition_samples) < required:
            self._reason = (
                "reacquiring selected target: "
                f"{len(self._reacquisition_samples)}/{required} stable scans"
            )
            self._publish_status()
            return
        corrected = (
            median(item[0] for item in self._reacquisition_samples),
            median(item[1] for item in self._reacquisition_samples),
        )
        self._rebuild_after_reacquisition(
            pose,
            corrected,
            association_error,
            execution_from_reference,
        )

    def _rebuild_after_reacquisition(
        self,
        pose: tuple[float, float, float],
        corrected: tuple[float, float],
        association_error: float,
        execution_from_reference: tuple[tuple[float, float], float],
    ) -> None:
        """Rebuild only the current cycle around a corrected target centre."""

        assert self._plan is not None
        old_plan = self._plan
        old_target = old_plan.selection.target
        maximum = self._float("target_reacquisition_maximum_correction")
        if not math.isfinite(association_error) or association_error > maximum:
            self._abort(
                "live-frame target association is too large: "
                f"{association_error:.3f} m"
            )
            return
        execution_shift = math.dist((old_target.x, old_target.y), corrected)
        if self._snapshot is None:
            self._abort("locked target snapshot is unavailable")
            return
        refreshed_targets = tuple(
            Target(
                candidate_id=item.candidate_id,
                x=position[0],
                y=position[1],
                color=item.color,
                radius=item.radius,
            )
            for item in self._snapshot.objects
            if item.state == "validated"
            for position in (
                transform_point_2d(
                    (item.position.x, item.position.y),
                    *execution_from_reference,
                ),
            )
        )
        matching_targets = tuple(
            item
            for item in refreshed_targets
            if item.candidate_id == old_target.candidate_id
        )
        if len(matching_targets) != 1:
            self._abort("selected target is absent from the locked plan")
            return
        target = replace(matching_targets[0], x=corrected[0], y=corrected[1])
        targets = tuple(
            target if item.candidate_id == target.candidate_id else item
            for item in refreshed_targets
        )
        selection = replace(old_plan.selection, target=target)
        try:
            local_plan = build_direct_cycle(
                targets,
                pose,
                old_plan.destination,
                self._float("staging_clearance"),
                self._float("contact_offset"),
                self._float("release_distance"),
                self._float("transit_clearance"),
                selection=selection,
                # The corrected leg starts at the already reached staging
                # pose. Return to that pose first; the original, successfully
                # traversed approach is stitched back to the task home below.
                home=pose,
                field_center=old_plan.field_center,
                require_clear_corridors=self._bool("require_clear_corridors"),
                allow_local_approach_inside_keepout=True,
            )
            plan = replace(
                local_plan,
                home=old_plan.home,
                return_path=stitch_reacquired_return_path(
                    local_plan.return_path,
                    old_plan.approach_path,
                ),
            )
        except ValueError as error:
            self._abort(f"target-corrected path is invalid: {error}")
            return
        self._plan = plan
        self._plan_targets = targets
        self._target_reacquired = True
        self._publish_plan(plan)
        self.get_logger().info(
            f"Reacquired candidate {target.candidate_id}; "
            f"live association error {association_error:.3f} m; "
            f"execution-frame shift {execution_shift:.3f} m"
        )
        self._start_phase("aligning_approach", pose[:2])

    def _tick_drive(
        self,
        pose: tuple[float, float, float],
        path: tuple[tuple[float, float], ...],
        speed: float,
        tolerance: float,
        timeout: float,
        next_state: str,
        deviation_guard: bool = True,
    ) -> None:
        if self._now() - self._phase_started > timeout:
            self._abort(f"{self._state} timed out")
            return
        endpoint = path[-1]
        if math.dist(pose[:2], endpoint) <= tolerance:
            self._publish_zero()
            self._start_phase(next_state, pose[:2])
            return
        target, deviation = polyline_tracking_target(
            path, pose[:2], self._float("path_lookahead")
        )
        if deviation_guard and deviation > self._float("maximum_path_deviation"):
            self._abort(f"{self._state} path deviation is {deviation:.3f} m")
            return
        desired = math.atan2(target[1] - pose[1], target[0] - pose[0])
        error = heading_error(pose[2], desired)
        linear, angular = pure_pursuit_command(
            speed,
            self._float("minimum_linear_speed"),
            error,
            max(1e-6, math.dist(pose[:2], target)),
            self._float("drive_heading_limit"),
            self._float("maximum_angular_speed"),
        )
        self._publish_command(linear, angular)

    def _tick_contact(self, pose: tuple[float, float, float]) -> None:
        if self._now() - self._phase_started > self._float("contact_timeout"):
            self._abort("contact approach timed out")
            return
        if (
            self._live_obstacle_stop_enabled()
            and self._unexpected_front_obstacle(
                self._float("target_acquire_distance")
            )
        ):
            self._abort("unexpected obstacle entered the contact corridor")
            return
        assert self._plan is not None
        self._tick_drive(
            pose,
            self._plan.contact_path,
            self._float("contact_speed"),
            self._float("contact_tolerance"),
            self._float("contact_timeout"),
            "pushing",
        )

    def _tick_push(self, pose: tuple[float, float, float]) -> None:
        if (
            self._live_obstacle_stop_enabled()
            and self._unexpected_front_obstacle(
                self._float("target_maximum_distance")
            )
        ):
            self._abort("unexpected obstacle entered the push corridor")
            return
        assert self._plan is not None
        before = self._state
        self._tick_drive(
            pose,
            self._plan.push_path,
            self._float("push_speed"),
            self._float("push_tolerance"),
            self._float("push_timeout"),
            "releasing",
        )
        if before != self._state and self._state == "releasing":
            # Release distance is physical wheel travel, not displacement in
            # the SLAM map.  A map correction while reversing must not satisfy
            # the 15 cm retreat or look like lateral/heading drift.
            release_pose = self._robot_pose_in_frame(self._string("odom_frame"))
            if release_pose is None:
                self._abort("odometry transform is unavailable at release")
                return
            self._release_origin = release_pose

    def _tick_release(self, pose: tuple[float, float, float]) -> None:
        if self._now() - self._phase_started > self._float("release_timeout"):
            self._abort("release retreat timed out")
            return
        if self._live_obstacle_stop_enabled() and self._rear_obstacle():
            self._abort("obstacle detected behind the robot")
            return
        if self._release_origin is None:
            self._abort("release origin is missing")
            return
        odom_pose = self._robot_pose_in_frame(self._string("odom_frame"))
        if odom_pose is None:
            self._tf_failures += 1
            if self._tf_failures >= self._int("transform_failure_limit"):
                self._abort("odometry transform was lost during release")
            return
        self._tf_failures = 0
        reverse, lateral, heading = retreat_motion(
            *self._release_origin,
            *odom_pose,
        )
        if (
            self._bool("release_forward_motion_guard_enabled")
            and reverse < -0.02
        ):
            self._abort("robot moved forward during release")
            return
        if abs(lateral) > 0.05 or abs(heading) > 0.15:
            self._abort("robot drifted during release")
            return
        if reverse >= self._float("release_distance"):
            self._publish_zero()
            self._start_phase("aligning_return", pose[:2])
            return
        linear = enforce_minimum_linear_speed(
            -self._float("release_speed"), self._float("minimum_linear_speed")
        )
        self._publish_command(linear, 0.0)

    def _tick_return(self, pose: tuple[float, float, float]) -> None:
        assert self._plan is not None
        self._tick_drive(
            pose,
            self._plan.return_path,
            self._float("return_speed"),
            self._float("home_tolerance"),
            self._float("return_timeout"),
            "aligning_home",
            deviation_guard=self._bool("return_path_deviation_guard_enabled"),
        )

    def _tick_yaw(
        self,
        pose: tuple[float, float, float],
        desired: float,
        *,
        heading_stable: bool = True,
    ) -> None:
        error = heading_error(pose[2], desired)
        if abs(error) > self._float("heading_tolerance"):
            self._heading_settle_cycles = 0
            if self._now() - self._phase_started > self._float("alignment_timeout"):
                self._abort("home heading alignment timed out")
                return
            self._publish_command(0.0, self._bounded_angular(error))
            return
        self._publish_zero()
        if not heading_stable:
            self._heading_settle_cycles = 0
            return
        if self._bool("inertial_heading_enabled"):
            angular_rate = self._inertial_rate()
            if angular_rate is None or not heading_is_settled(
                error,
                angular_rate,
                self._float("heading_tolerance"),
                self._float("heading_settle_rate"),
            ):
                self._heading_settle_cycles = 0
                return
            self._heading_settle_cycles += 1
            if self._heading_settle_cycles < self._int(
                "heading_settle_required_cycles"
            ):
                return
        self._heading_settle_cycles = 0
        self._begin_finalize()

    def _begin_finalize(self) -> None:
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
            self._abort("perception reset services are unavailable after return")
            return
        destinations = self._delivered_destinations + [self._plan.destination]
        try:
            exclusion_centers = self._transform_points(
                destinations,
                self._string("fixed_frame"),
                self._string("execution_frame"),
            )
        except ValueError as error:
            self._abort(str(error))
            return
        exclusions = SetSnapshotExclusions.Request()
        exclusions.centers = [
            Point(x=x, y=y, z=0.0) for x, y in exclusion_centers
        ]
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
        self._snapshot = None
        self._expected_inventory = (tuple(colors), tuple(counts))
        final_state = "task_complete" if sum(counts) == 0 else "collecting_remaining"
        final_reason = (
            "all configured cylinders delivered"
            if sum(counts) == 0
            else "push complete; collecting the reduced inventory"
        )
        self._state = "finalizing"
        self._reason = "resetting perception after a completed push"
        self._publish_status()
        self._run_service_steps(steps, final_state, final_reason)

    def _run_service_steps(self, steps, final_state: str, final_reason: str) -> None:
        self._generation += 1
        generation = self._generation
        self._operation_busy = True

        def run(index: int) -> None:
            if generation != self._generation:
                return
            if index == len(steps):
                self._operation_busy = False
                if final_state in ("collecting", "collecting_remaining"):
                    self._snapshot = None
                self._state = final_state
                self._reason = final_reason
                self._plan = None
                if final_state == "task_complete":
                    self._run_all_active = False
                self._publish_status()
                return
            label, client, request = steps[index]
            if label == "snapshot start":
                # Establish the generation boundary before the collection
                # service can publish its first result.
                self._collection_started = self._now()
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

    def _start_phase(self, state: str, start: tuple[float, float]) -> None:
        self._state = state
        if state == "aligning_home":
            self._heading_settle_cycles = 0
            self._home_heading_samples.clear()
            self._current_fixed_yaw = None
            self._current_map_to_odom_yaw = None
            self._home_heading_tf_failures = 0
        self._reason = f"compact direct cycle: {state}"
        self._phase_started = self._now()
        del start
        self._publish_status()

    def _robot_pose(self) -> tuple[float, float, float] | None:
        return self._robot_pose_in_frame(self._string("execution_frame"))

    def _robot_pose_in_frame(
        self, frame: str
    ) -> tuple[float, float, float] | None:
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

    def _robot_yaw_in_frame(self, frame: str) -> float | None:
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
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )

    def _start_or_resume_inertial_heading(self, reference_yaw: float) -> None:
        with self._inertial_lock:
            if self._latest_imu_sample is None or self._imu_stale_locked():
                raise ValueError("IMU angular-rate input is missing or stale")
            stamp, angular_rate = self._latest_imu_sample
            bias = (
                median(self._imu_bias_samples) if self._imu_bias_samples else 0.0
            )
            if not self._inertial_heading.initialized:
                self._inertial_heading.reset(
                    reference_yaw,
                    stamp,
                    angular_rate,
                    bias,
                )
            else:
                self._inertial_heading.bias = bias
                self._inertial_heading.observe(
                    stamp,
                    angular_rate,
                    integrate=False,
                )
            self._imu_gap_invalid = False

    def _inertial_yaw(self) -> float | None:
        with self._inertial_lock:
            return self._inertial_heading.yaw

    def _inertial_rate(self) -> float | None:
        with self._inertial_lock:
            if self._latest_imu_sample is None:
                return None
            return self._latest_imu_sample[1] - self._inertial_heading.bias

    def _imu_stale(self) -> bool:
        with self._inertial_lock:
            return self._imu_stale_locked()

    def _imu_stale_locked(self) -> bool:
        return (
            self._last_imu_received <= 0.0
            or self._now() - self._last_imu_received > self._float("imu_timeout")
        )

    def _planar_transform(
        self,
        target_frame: str,
        source_frame: str,
        *,
        at_time: Time | None = None,
    ) -> tuple[tuple[float, float], float]:
        if target_frame == source_frame:
            return (0.0, 0.0), 0.0
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                at_time if at_time is not None else Time(),
                timeout=Duration(seconds=self._float("transform_timeout")),
            ).transform
        except TransformException as error:
            raise ValueError(
                f"transform {source_frame} to {target_frame} is unavailable: {error}"
            ) from error
        rotation = transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        return (transform.translation.x, transform.translation.y), yaw

    def _transform_points(
        self,
        points: list[tuple[float, float]],
        target_frame: str,
        source_frame: str,
    ) -> list[tuple[float, float]]:
        translation, rotation = self._planar_transform(target_frame, source_frame)
        return [
            transform_point_2d(point, translation, rotation) for point in points
        ]

    def _scan_age(self) -> float:
        if self._scan is None:
            return math.inf
        return max(0.0, self._now() - self._last_scan_received)

    def _scan_stale(self, timeout: float | None = None) -> bool:
        limit = self._float("scan_timeout") if timeout is None else timeout
        return self._scan is None or self._scan_age() > limit

    def _odometry_stale(self) -> bool:
        return self._odom_sample is None or (
            self._now() - self._last_odom_received
            > self._float("odometry_timeout")
        )

    def _target_present(self, maximum_distance: float | None = None) -> bool:
        if self._scan is None:
            return False
        scan = self._scan
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

    def _unexpected_front_obstacle(self, target_depth: float) -> bool:
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

    def _live_obstacle_stop_enabled(self) -> bool:
        """Return whether live scan returns may stop a prevalidated route."""

        return self._bool("live_obstacle_stop_enabled")

    def _front_obstacle(self) -> bool:
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
            1e-6,
            1e-6,
        )

    def _rear_obstacle(self) -> bool:
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
        frame = self._string("execution_frame")
        stamp = self.get_clock().now().to_msg()
        self._approach_publisher.publish(self._path(frame, stamp, plan.approach_path))
        self._contact_publisher.publish(self._path(frame, stamp, plan.contact_path))
        self._push_publisher.publish(self._path(frame, stamp, plan.push_path))
        self._return_publisher.publish(self._path(frame, stamp, plan.return_path))
        clear = Marker(action=Marker.DELETEALL)
        target = Marker()
        target.header.frame_id = frame
        target.header.stamp = stamp
        target.ns = "compact_selected_target"
        target.id = int(plan.selection.target.candidate_id)
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = plan.selection.target.x
        target.pose.position.y = plan.selection.target.y
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.12
        target.color.r = 1.0
        target.color.a = 1.0
        markers = [clear, target]
        if plan.keepout_boundary:
            keepout = Marker()
            keepout.header.frame_id = frame
            keepout.header.stamp = stamp
            keepout.ns = "compact_remaining_keepout"
            keepout.id = 0
            keepout.type = Marker.LINE_STRIP
            keepout.action = Marker.ADD
            keepout.pose.orientation.w = 1.0
            keepout.scale.x = 0.025
            keepout.color.r = 1.0
            keepout.color.g = 0.55
            keepout.color.a = 0.9
            keepout.points = [
                Point(x=x, y=y, z=0.02)
                for x, y in (*plan.keepout_boundary, plan.keepout_boundary[0])
            ]
            markers.append(keepout)
        self._marker_publisher.publish(MarkerArray(markers=markers))

    @staticmethod
    def _path(frame: str, stamp, points) -> Path:
        path = Path()
        path.header.frame_id = frame
        path.header.stamp = stamp
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _publish_command(self, linear: float, angular: float) -> None:
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self._cmd_publisher.publish(command)

    def _publish_zero(self) -> None:
        self._cmd_publisher.publish(Twist())

    def _terminate(self, state: str, reason: str, *, clear_plan: bool = False) -> None:
        self._state = state
        self._reason = reason
        self._stop_burst = 5
        self._publish_zero()
        if clear_plan:
            self._snapshot = None
            self._plan = None
            self._plan_targets = ()
            self._target_reacquired = False
            self._reacquisition_reference_frame = ""
            self._reacquisition_reference_point = None
            self._reacquisition_samples.clear()
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
            "architecture": "compact_direct",
            "execution_enabled": self._bool("execution_enabled"),
            "nav2_used": False,
            "execution_frame": self._string("execution_frame"),
            "heading_source": (
                "map_path_with_fixed_frame_home_alignment_and_imu_settle"
                if self._bool("absolute_home_heading_enabled")
                else (
                    "map_path_with_imu_home_alignment"
                    if self._bool("inertial_heading_enabled")
                    else "execution_frame_tf"
                )
            ),
            "state": self._state,
            "reason": self._reason,
            "target_id": (
                plan.selection.target.candidate_id if plan is not None else -1
            ),
            "target_color": (
                plan.selection.target.color if plan is not None else ""
            ),
            "destination": list(plan.destination) if plan is not None else None,
            "delivered_count": len(self._delivered_destinations),
            "run_all_active": self._run_all_active,
            "home_map_yaw": self._task_home_fixed_yaw,
            "current_map_yaw": self._current_fixed_yaw,
            "home_heading_error": (
                heading_error(
                    self._current_fixed_yaw,
                    self._task_home_fixed_yaw,
                )
                if self._current_fixed_yaw is not None
                and self._task_home_fixed_yaw is not None
                else None
            ),
            "imu_integrated_yaw": self._inertial_yaw(),
            "map_to_odom_yaw": self._current_map_to_odom_yaw,
        }
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _services_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in (
                self._validation_reset,
                self._snapshot_reset,
                self._snapshot_start,
                self._inventory_update,
                self._exclusion_update,
            )
        )

    def _failure(self, response, reason: str):
        response.success = False
        response.message = reason
        return response

    def _bounded_angular(self, error: float) -> float:
        limit = self._float("maximum_angular_speed")
        return max(-limit, min(limit, self._float("angular_gain") * error))

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

    def prepare_shutdown(self) -> None:
        self._generation += 1
        self._operation_busy = False
        self._run_all_active = False
        self._state = "cancelled"
        for _unused in range(5):
            self._publish_zero()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DirectTaskControllerNode()
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
