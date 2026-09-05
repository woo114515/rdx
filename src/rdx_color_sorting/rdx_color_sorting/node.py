"""ROS 2 adapter for the independent color-sorting demo."""

from __future__ import annotations

import json
import importlib
import math
from typing import Any

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import String

from .behavior import (
    ControllerConfig,
    Detection,
    frame_transform,
    Pose2D,
    SortController,
    VelocityCommand,
    transform_pose,
    Zone,
)
from .vision import ColorDetector, HsvRange
from .ranging import range_near_bearing


class ColorSortingNode(Node):
    """Detect green, blue, and orange objects and request safe motion."""

    def __init__(self) -> None:
        super().__init__("color_sorting")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._detections: tuple[Detection, ...] = ()
        self._pose: Pose2D | None = None
        self._odom_pose: Pose2D | None = None
        self._map_from_odom: Pose2D | None = None
        self._last_image_time: float | None = None
        self._last_odom_time: float | None = None
        self._last_scan: LaserScan | None = None
        self._last_scan_time: float | None = None
        self._pose_source = str(self.get_parameter("pose_source").value)
        if self._pose_source not in {"odom", "amcl"}:
            raise ValueError("pose_source must be 'odom' or 'amcl'")
        self._motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self._image_timeout = float(self.get_parameter("image_timeout").value)
        self._odom_timeout = float(self.get_parameter("odom_timeout").value)
        self._lidar_target_range_enabled = bool(
            self.get_parameter("lidar_target_range_enabled").value
        )
        self._target_scan_timeout = float(
            self.get_parameter("target_scan_timeout").value
        )
        self._nav2_return_enabled = bool(
            self.get_parameter("nav2_return_enabled").value
        )
        zones_relative = bool(
            self.get_parameter("zones_relative_to_origin").value
        )
        if self._nav2_return_enabled and (
            self._pose_source != "amcl" or zones_relative
        ):
            raise ValueError(
                "nav2 return requires pose_source=amcl and absolute map zones"
            )
        self._nav2_client: Any | None = None
        self._nav2_action_type: Any | None = None
        self._nav2_goal_requested = False
        self._nav2_goal_handle: Any | None = None
        self._navigation_state = "disabled"
        if self._nav2_return_enabled:
            try:
                action_module = importlib.import_module("nav2_msgs.action")
            except ImportError as error:
                raise RuntimeError(
                    "nav2_return_enabled requires the nav2_msgs package"
                ) from error
            action_client_module = importlib.import_module("rclpy.action")
            self._nav2_action_type = action_module.NavigateToPose
            self._nav2_client = action_client_module.ActionClient(
                self,
                self._nav2_action_type,
                str(self.get_parameter("nav2_action_name").value),
            )
            self._navigation_state = "idle"

        self._detector = ColorDetector(
            ranges=self._load_hsv_ranges(),
            minimum_area_ratio=float(
                self.get_parameter("minimum_area_ratio").value
            ),
            maximum_area_ratio=float(
                self.get_parameter("maximum_area_ratio").value
            ),
            roi_top_ratio=float(self.get_parameter("roi_top_ratio").value),
            minimum_width_height_ratio=float(
                self.get_parameter("minimum_width_height_ratio").value
            ),
            maximum_width_height_ratio=float(
                self.get_parameter("maximum_width_height_ratio").value
            ),
            minimum_extent=float(self.get_parameter("minimum_extent").value),
            minimum_solidity=float(
                self.get_parameter("minimum_solidity").value
            ),
            bottle_detection_enabled=bool(
                self.get_parameter("bottle_detection_enabled").value
            ),
            bottle_minimum_width_height_ratio=float(
                self.get_parameter(
                    "bottle_minimum_width_height_ratio"
                ).value
            ),
            bottle_maximum_width_height_ratio=float(
                self.get_parameter(
                    "bottle_maximum_width_height_ratio"
                ).value
            ),
            bottle_minimum_extent=float(
                self.get_parameter("bottle_minimum_extent").value
            ),
            bottle_minimum_solidity=float(
                self.get_parameter("bottle_minimum_solidity").value
            ),
            bottle_maximum_neck_body_width_ratio=float(
                self.get_parameter(
                    "bottle_maximum_neck_body_width_ratio"
                ).value
            ),
        )
        colors = tuple(self.get_parameter("color_order").value)
        self._controller = SortController(
            zones={
                color: Zone(
                    x=float(self.get_parameter(f"zones.{color}.x").value),
                    y=float(self.get_parameter(f"zones.{color}.y").value),
                )
                for color in colors
            },
            targets_per_color={
                color: int(
                    self.get_parameter(f"targets_per_color.{color}").value
                )
                for color in colors
            },
            color_order=colors,
            config=self._load_controller_config(),
            zones_relative_to_origin=zones_relative,
            external_return_enabled=self._nav2_return_enabled,
        )

        output_topic = str(self.get_parameter("command_output_topic").value)
        self._command_publisher = self.create_publisher(Twist, output_topic, 10)
        self._status_publisher = self.create_publisher(
            String,
            "/rdx_sorting/status",
            10,
        )
        self._debug_publisher = self.create_publisher(
            CompressedImage,
            "/rdx_sorting/debug/compressed",
            2,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        if self._pose_source == "amcl":
            self.create_subscription(
                PoseWithCovarianceStamped,
                str(self.get_parameter("localization_pose_topic").value),
                self._on_localization_pose,
                10,
            )
        if self._lidar_target_range_enabled:
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_topic").value),
                self._on_scan,
                10,
            )

        image_topic = str(self.get_parameter("image_topic").value)
        if bool(self.get_parameter("compressed_input").value):
            self.create_subscription(
                CompressedImage,
                image_topic,
                self._on_compressed_image,
                2,
            )
        else:
            self.create_subscription(Image, image_topic, self._on_image, 2)

        control_rate = float(self.get_parameter("control_rate").value)
        if control_rate <= 0.0:
            raise ValueError("control_rate must be positive")
        self.create_timer(1.0 / control_rate, self._on_control_timer)
        self.get_logger().info(
            f"Color sorting ready; motion_enabled={self._motion_enabled}, "
            f"output={output_topic}"
        )

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "motion_enabled": False,
            "image_topic": "/image_raw",
            "compressed_input": False,
            "odom_topic": "/odom_raw",
            "pose_source": "odom",
            "localization_pose_topic": "/amcl_pose",
            "zones_relative_to_origin": True,
            "nav2_return_enabled": False,
            "nav2_action_name": "/navigate_to_pose",
            "map_frame": "map",
            "command_output_topic": "/rdx_sorting/cmd_vel_request",
            "control_rate": 10.0,
            "image_timeout": 0.5,
            "odom_timeout": 0.5,
            "lidar_target_range_enabled": False,
            "scan_topic": "/scan",
            "target_scan_timeout": 0.5,
            "camera_horizontal_fov_degrees": 62.0,
            "camera_lidar_yaw_offset_degrees": 0.0,
            "target_range_half_angle_degrees": 3.0,
            "target_range_minimum_samples": 3,
            "roi_top_ratio": 0.25,
            "minimum_area_ratio": 0.001,
            "maximum_area_ratio": 0.35,
            "minimum_width_height_ratio": 0.25,
            "maximum_width_height_ratio": 0.90,
            "minimum_extent": 0.55,
            "minimum_solidity": 0.88,
            "bottle_detection_enabled": True,
            "bottle_minimum_width_height_ratio": 0.15,
            "bottle_maximum_width_height_ratio": 0.65,
            "bottle_minimum_extent": 0.35,
            "bottle_minimum_solidity": 0.75,
            "bottle_maximum_neck_body_width_ratio": 0.78,
            "color_order": ["green", "blue", "orange"],
            "targets_per_color.green": 2,
            "targets_per_color.blue": 2,
            "targets_per_color.orange": 2,
            "zones.green.x": 1.2,
            "zones.green.y": 0.8,
            "zones.blue.x": 1.2,
            "zones.blue.y": 0.0,
            "zones.orange.x": 1.2,
            "zones.orange.y": -0.8,
            "hsv.green.lower": [35, 80, 60],
            "hsv.green.upper": [85, 255, 255],
            "hsv.blue.lower": [90, 80, 50],
            "hsv.blue.upper": [135, 255, 255],
            "hsv.orange.lower": [0, 80, 60],
            "hsv.orange.upper": [25, 255, 255],
            "search_angular_speed": 0.18,
            "maximum_search_duration": 40.0,
            "search_timeout_is_fault": False,
            "finish_when_no_target_duration": 0.0,
            "minimum_detection_duration": 0.25,
            "align_kp": 0.45,
            "max_align_speed": 0.25,
            "align_tolerance": 0.08,
            "approach_speed": 0.08,
            "minimum_approach_speed": 0.03,
            "approach_range_kp": 0.40,
            "contact_distance": 0.24,
            "contact_area_ratio": 0.12,
            "target_lost_timeout": 0.6,
            "push_speed": 0.08,
            "maximum_push_duration": 20.0,
            "push_target_lost_timeout": 0.0,
            "push_kp": 0.7,
            "yaw_kp": 0.8,
            "max_yaw_speed": 0.20,
            "zone_tolerance": 0.12,
            "release_speed": 0.06,
            "release_duration": 1.0,
            "delivery_confirmation_timeout": 0.0,
            "delivery_confirmation_duration": 0.25,
            "delivery_maximum_area_ratio": 0.15,
            "delivery_maximum_area_scale": 0.85,
            "delivery_maximum_x_error": 0.35,
            "return_speed": 0.10,
            "return_kp": 0.7,
            "return_tolerance": 0.12,
            "return_corridor_offset": 0.80,
            "return_corridor_margin": 0.25,
            "maximum_return_duration": 35.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_hsv_ranges(self) -> dict[str, tuple[HsvRange, ...]]:
        result: dict[str, tuple[HsvRange, ...]] = {}
        for color in self.get_parameter("color_order").value:
            lower = tuple(
                int(value)
                for value in self.get_parameter(f"hsv.{color}.lower").value
            )
            upper = tuple(
                int(value)
                for value in self.get_parameter(f"hsv.{color}.upper").value
            )
            result[color] = (HsvRange(lower=lower, upper=upper),)
        return result

    def _load_controller_config(self) -> ControllerConfig:
        fields = ControllerConfig.__dataclass_fields__
        values: dict[str, Any] = {}
        for name, field in fields.items():
            value = self.get_parameter(name).value
            values[name] = (
                bool(value) if isinstance(field.default, bool) else float(value)
            )
        return ControllerConfig(
            **values,
        )

    def _on_image(self, message: Image) -> None:
        if message.encoding.lower() == "nv12":
            frame = np.frombuffer(message.data, dtype=np.uint8)
            expected_size = message.width * message.height * 3 // 2
            if frame.size != expected_size:
                self.get_logger().error(
                    "Invalid NV12 frame size: "
                    f"expected {expected_size}, received {frame.size}"
                )
                return
            frame = frame.reshape(message.height * 3 // 2, message.width)
            image = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_NV12)
        else:
            image = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        self._process_image(image)

    def _on_compressed_image(self, message: CompressedImage) -> None:
        image = self._bridge.compressed_imgmsg_to_cv2(
            message,
            desired_encoding="bgr8",
        )
        self._process_image(image)

    def _process_image(self, image: Any) -> None:
        result = self._detector.detect(image)
        self._detections = tuple(
            Detection(
                color=detection.color,
                x_error=detection.x_error,
                area_ratio=detection.area_ratio,
                range_m=self._target_range(detection),
                shape=detection.shape,
            )
            for detection in result.detections
        )
        self._last_image_time = self._now()
        success, encoded = cv2.imencode(".jpg", result.debug_image)
        if success:
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.format = "jpeg"
            message.data = encoded.tobytes()
            self._debug_publisher.publish(message)

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan = message
        self._last_scan_time = self._now()

    def _target_range(self, detection: Detection) -> float | None:
        if not self._lidar_target_range_enabled:
            return None
        if self._last_scan is None or self._last_scan_time is None:
            return None
        if self._now() - self._last_scan_time > self._target_scan_timeout:
            return None
        scan = self._last_scan
        half_fov = math.radians(
            float(self.get_parameter("camera_horizontal_fov_degrees").value)
        ) / 2.0
        yaw_offset = math.radians(
            float(
                self.get_parameter("camera_lidar_yaw_offset_degrees").value
            )
        )
        bearing = yaw_offset - detection.x_error * half_fov
        return range_near_bearing(
            scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            bearing=bearing,
            half_angle=math.radians(
                float(
                    self.get_parameter(
                        "target_range_half_angle_degrees"
                    ).value
                )
            ),
            minimum_samples=int(
                self.get_parameter("target_range_minimum_samples").value
            ),
        )

    def _on_odom(self, message: Odometry) -> None:
        self._odom_pose = self._pose_from_message(message.pose.pose)
        if self._pose_source == "odom":
            self._pose = self._odom_pose
        elif self._map_from_odom is not None:
            self._pose = transform_pose(
                self._map_from_odom,
                self._odom_pose,
            )
        self._last_odom_time = self._now()

    def _on_localization_pose(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        if self._odom_pose is None:
            return
        map_pose = self._pose_from_message(message.pose.pose)
        self._map_from_odom = frame_transform(map_pose, self._odom_pose)
        self._pose = transform_pose(
            self._map_from_odom,
            self._odom_pose,
        )

    @staticmethod
    def _pose_from_message(message: Any) -> Pose2D:
        orientation = message.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        return Pose2D(message.position.x, message.position.y, yaw)

    def _on_control_timer(self) -> None:
        now = self._now()
        if not self._motion_enabled:
            self._publish_command(VelocityCommand())
            self._publish_status("preview")
            return
        if (
            self._last_image_time is None
            or now - self._last_image_time > self._image_timeout
        ):
            self._publish_command(VelocityCommand())
            self._publish_status("stopped: stale image")
            return
        if (
            self._last_odom_time is None
            or now - self._last_odom_time > self._odom_timeout
        ):
            self._publish_command(VelocityCommand())
            self._publish_status("stopped: stale odometry")
            return
        if self._pose_source == "amcl" and self._map_from_odom is None:
            self._publish_command(VelocityCommand())
            self._publish_status("stopped: localization unavailable")
            return
        command = self._controller.update(now, self._detections, self._pose)
        if self._nav2_return_enabled and self._controller.state.value == "return":
            command = VelocityCommand()
            self._start_nav2_return()
        self._publish_command(command)
        self._publish_status(self._controller.state.value)

    def _start_nav2_return(self) -> None:
        if self._nav2_goal_requested:
            return
        if self._nav2_client is None or self._nav2_action_type is None:
            self._controller.stop_with_fault("nav2 client unavailable")
            return
        if not self._nav2_client.server_is_ready():
            self._navigation_state = "waiting for nav2 server"
            return
        origin = self._controller.origin
        if origin is None:
            self._controller.stop_with_fault("map origin unavailable")
            return

        goal = self._nav2_action_type.Goal()
        goal.pose.header.frame_id = str(self.get_parameter("map_frame").value)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = origin.x
        goal.pose.pose.position.y = origin.y
        goal.pose.pose.orientation.z = math.sin(origin.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(origin.yaw / 2.0)
        self._nav2_goal_requested = True
        self._navigation_state = "goal pending"
        future = self._nav2_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav2_goal_response)

    def _on_nav2_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # ROS future transports runtime failures.
            self._navigation_state = "goal request failed"
            self._controller.stop_with_fault(f"nav2 goal request failed: {error}")
            return
        if not goal_handle.accepted:
            self._navigation_state = "goal rejected"
            self._controller.stop_with_fault("nav2 return goal rejected")
            return
        self._nav2_goal_handle = goal_handle
        self._navigation_state = "navigating"
        future = goal_handle.get_result_async()
        future.add_done_callback(self._on_nav2_result)

    def _on_nav2_result(self, future: Any) -> None:
        self._nav2_goal_handle = None
        try:
            result = future.result()
        except Exception as error:  # ROS future transports runtime failures.
            self._navigation_state = "navigation failed"
            self._controller.stop_with_fault(f"nav2 return failed: {error}")
            return
        if result.status != 4:  # action_msgs/msg/GoalStatus.STATUS_SUCCEEDED
            self._navigation_state = f"navigation failed ({result.status})"
            self._controller.stop_with_fault(
                f"nav2 return failed with status {result.status}"
            )
            return
        self._navigation_state = "arrived"
        self._nav2_goal_requested = False
        self._controller.complete_external_return(self._now())

    def _publish_command(self, command: VelocityCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.angular.z = command.angular_z
        self._command_publisher.publish(message)

    def _publish_status(self, reason: str) -> None:
        message = String()
        message.data = json.dumps(
            {
                "state": reason,
                "fault_reason": self._controller.fault_reason,
                "current_color": self._controller.current_color,
                "remaining": self._controller.remaining,
                "processed": self._controller.processed,
                "pose_source": self._pose_source,
                "navigation": self._navigation_state,
                "detections": [
                    {
                        "color": item.color,
                        "shape": item.shape,
                        "x_error": round(item.x_error, 4),
                        "area_ratio": round(item.area_ratio, 5),
                        "range_m": (
                            None
                            if item.range_m is None
                            else round(item.range_m, 3)
                        ),
                    }
                    for item in self._detections
                ],
            },
            ensure_ascii=False,
        )
        self._status_publisher.publish(message)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def destroy_node(self) -> bool:
        if self._nav2_goal_handle is not None:
            self._nav2_goal_handle.cancel_goal_async()
        self._publish_command(VelocityCommand())
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ColorSortingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
