"""Switch Task 3 exactly once from GMapping to a captured map and AMCL."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from color_object_sorter_interfaces.msg import ValidatedCylinderArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .fixed_map import atomic_write_map, pose_distance


class FixedLocalizationHandoff(Node):
    """Capture the initial map and transfer map->odom ownership to AMCL."""

    def __init__(self) -> None:
        super().__init__("task3_localization_handoff")
        defaults = {
            "validated_topic": "/cylinder_snapshot/validated_objects",
            "dynamic_map_topic": "/map",
            "fixed_map_topic": "/task3/fixed_map",
            "initial_pose_topic": "/initialpose",
            "amcl_pose_topic": "/amcl_pose",
            "gmapping_tracking_service": "/slam_gmapping/set_tracking_enabled",
            "fixed_frame": "map",
            "robot_frame": "base_footprint",
            "map_directory": "/home/sunrise/task3_maps",
            "map_basename": "task3_initial",
            "automatic_handoff": True,
            "transform_timeout": 0.20,
            "handoff_settle_time": 0.60,
            "amcl_timeout": 15.0,
            "maximum_handoff_position_error": 0.20,
            "maximum_handoff_yaw_error": 0.20,
            "initial_position_variance": 0.01,
            "initial_yaw_variance": 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._fixed_map_publisher = self.create_publisher(
            OccupancyGrid, self._string("fixed_map_topic"), latched
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self._string("initial_pose_topic"), 10
        )
        self._status_publisher = self.create_publisher(
            String, "/simple_task3/localization_status", latched
        )
        self.create_subscription(
            OccupancyGrid,
            self._string("dynamic_map_topic"),
            self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.create_subscription(
            ValidatedCylinderArray,
            self._string("validated_topic"),
            self._on_snapshot,
            latched,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._string("amcl_pose_topic"),
            self._on_amcl_pose,
            10,
        )
        self.create_service(Trigger, "/simple_task3/localization_handoff", self._trigger)
        self._tracking_client = self.create_client(
            SetBool,
            self._string("gmapping_tracking_service"),
            callback_group=ReentrantCallbackGroup(),
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._latest_map: OccupancyGrid | None = None
        self._initial_snapshot_seen = False
        self._handoff_requested = False
        self._handoff_pose: tuple[float, float, float] | None = None
        self._handoff_started = False
        self._tracking_disabled = False
        self._fixed_map: OccupancyGrid | None = None
        self._initial_pose: PoseWithCovarianceStamped | None = None
        self._disable_future = None
        self._disabled_at = 0.0
        self._amcl_deadline = 0.0
        self._ready = False
        self._state = "mapping"
        self._reason = "waiting for the initial locked cylinder snapshot"
        self.create_timer(0.10, self._tick)
        self._publish_status()

    def _on_map(self, message: OccupancyGrid) -> None:
        if not self._handoff_started:
            self._latest_map = message

    def _on_snapshot(self, message: ValidatedCylinderArray) -> None:
        if self._initial_snapshot_seen or not message.ready or not message.locked:
            return
        expected = sum(int(value) for value in message.expected_color_counts)
        validated = sum(item.state == "validated" for item in message.objects)
        if expected <= 0 or validated != expected:
            return
        if self._bool("automatic_handoff"):
            self._handoff_requested = True
        self._initial_snapshot_seen = True

    def _trigger(self, request, response):
        del request
        if self._ready:
            response.success = True
            response.message = "fixed-map AMCL localization is already ready"
            return response
        if self._handoff_started:
            response.success = False
            response.message = f"handoff is already in progress: {self._state}"
            return response
        self._handoff_requested = True
        response.success = True
        response.message = "fixed-map localization handoff requested"
        return response

    def _start_handoff(self) -> None:
        if self._latest_map is None:
            raise ValueError("a GMapping OccupancyGrid has not been received")
        pose = self._robot_pose()
        if pose is None:
            raise ValueError("map to base_footprint is unavailable")
        if not self._tracking_client.service_is_ready():
            raise ValueError("patched GMapping tracking service is unavailable")
        self._handoff_started = True
        self._handoff_pose = pose
        self._fixed_map = self._latest_map
        directory = Path(self._string("map_directory"))
        paths = atomic_write_map(
            directory, self._string("map_basename"), self._fixed_map
        )
        self._state = "disabling_gmapping"
        self._reason = f"fixed map saved to {paths[0]}"
        request = SetBool.Request()
        request.data = False
        self._disable_future = self._tracking_client.call_async(request)
        self._publish_status()

    def _tick(self) -> None:
        if self._handoff_requested and not self._handoff_started and not self._ready:
            try:
                self._start_handoff()
            except ValueError as error:
                self._reason = f"handoff waiting: {error}"
                self._publish_status()
            return
        if not self._handoff_started or self._ready:
            return
        if self._state == "disabling_gmapping":
            if self._disable_future is None or not self._disable_future.done():
                return
            try:
                result = self._disable_future.result()
            except Exception as error:
                self._fail(f"could not disable GMapping: {error}")
                return
            if not result.success:
                self._fail(f"GMapping rejected handoff: {result.message}")
                return
            self._tracking_disabled = True
            self._disabled_at = time.monotonic()
            self._state = "settling"
            self._reason = "GMapping scan matching and TF are disabled"
            self._publish_status()
            return
        if self._state == "settling":
            if time.monotonic() - self._disabled_at < self._float("handoff_settle_time"):
                return
            assert self._fixed_map is not None
            assert self._handoff_pose is not None
            self._fixed_map.header.stamp = self.get_clock().now().to_msg()
            self._fixed_map_publisher.publish(self._fixed_map)
            self._initial_pose = self._make_initial_pose(self._handoff_pose)
            self._amcl_deadline = time.monotonic() + self._float("amcl_timeout")
            self._state = "waiting_for_amcl"
            self._reason = "fixed map published; seeding AMCL"
            self._publish_status()
            return
        if self._state == "waiting_for_amcl":
            assert self._fixed_map is not None
            assert self._initial_pose is not None
            self._fixed_map.header.stamp = self.get_clock().now().to_msg()
            self._fixed_map_publisher.publish(self._fixed_map)
            self._initial_pose.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_publisher.publish(self._initial_pose)
            if time.monotonic() > self._amcl_deadline:
                self._fail("AMCL did not confirm the captured pose before timeout")

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        if self._state != "waiting_for_amcl" or self._handoff_pose is None:
            return
        pose = message.pose.pose
        yaw = math.atan2(
            2.0
            * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y
            ),
            1.0 - 2.0 * (pose.orientation.y**2 + pose.orientation.z**2),
        )
        error_position, error_yaw = pose_distance(
            self._handoff_pose, (pose.position.x, pose.position.y, yaw)
        )
        if error_position > self._float("maximum_handoff_position_error"):
            return
        if error_yaw > self._float("maximum_handoff_yaw_error"):
            return
        self._ready = True
        self._state = "fixed_amcl"
        self._reason = (
            f"AMCL owns map->odom; handoff error={error_position:.3f}m/"
            f"{math.degrees(error_yaw):.1f}deg"
        )
        self._publish_status()

    def _make_initial_pose(self, pose: tuple[float, float, float]):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self._string("fixed_frame")
        message.pose.pose.position.x = pose[0]
        message.pose.pose.position.y = pose[1]
        message.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        message.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        message.pose.covariance[0] = self._float("initial_position_variance")
        message.pose.covariance[7] = self._float("initial_position_variance")
        message.pose.covariance[35] = self._float("initial_yaw_variance")
        return message

    def _robot_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._string("fixed_frame"),
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

    def _fail(self, reason: str) -> None:
        self._state = "failed"
        self._reason = reason
        self._publish_status()
        self.get_logger().error(reason)

    def _publish_status(self) -> None:
        payload = {
            "ready": self._ready,
            "state": self._state,
            "reason": self._reason,
            "map_saved": self._fixed_map is not None,
            "gmapping_tracking_disabled": self._tracking_disabled,
            "localization_owner": (
                "amcl"
                if self._ready
                else "none"
                if self._tracking_disabled
                else "gmapping"
            ),
        }
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _bool(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedLocalizationHandoff()
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
