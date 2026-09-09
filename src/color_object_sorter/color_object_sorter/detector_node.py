"""ROS adapter for multi-object color perception."""

from __future__ import annotations

import json
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from color_object_sorter_interfaces.msg import ColorObject, ColorObjectArray
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .tracking import ObjectTracker
from .vision import ColorObjectDetector, HsvRange


class ColorObjectDetectorNode(Node):
    """Decode compressed images and publish every tracked color object."""

    def __init__(self) -> None:
        super().__init__("color_object_detector")
        self._declare_parameters()
        self._detector = ColorObjectDetector(
            self._load_ranges(),
            minimum_area_ratio=self._float("minimum_area_ratio"),
            maximum_area_ratio=self._float("maximum_area_ratio"),
            minimum_aspect_ratio=self._float("minimum_aspect_ratio"),
            maximum_aspect_ratio=self._float("maximum_aspect_ratio"),
            minimum_extent=self._float("minimum_extent"),
            minimum_solidity=self._float("minimum_solidity"),
            roi_top_ratio=self._float("roi_top_ratio"),
            morphology_kernel_size=self._int("morphology_kernel_size"),
        )
        self._tracker = ObjectTracker(
            maximum_distance=self._float("tracker_maximum_distance"),
            maximum_missed=self._int("tracker_maximum_missed"),
        )
        self._image_timeout = self._float("image_timeout")
        debug_fps = self._float("debug_fps")
        self._debug_quality = self._int("debug_jpeg_quality")
        if self._image_timeout <= 0.0 or debug_fps <= 0.0:
            raise ValueError("timeouts and rates must be positive")
        if not 1 <= self._debug_quality <= 100:
            raise ValueError("debug_jpeg_quality must be in [1, 100]")
        self._debug_period = 1.0 / debug_fps

        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._detections_publisher = self.create_publisher(
            ColorObjectArray, self._string("detections_topic"), 10
        )
        self._debug_publisher = self.create_publisher(
            CompressedImage, self._string("debug_topic"), debug_qos
        )
        self._health_publisher = self.create_publisher(
            String, self._string("health_topic"), 1
        )
        self.create_subscription(
            CompressedImage,
            self._string("image_topic"),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._publish_health)
        self._last_image_monotonic: float | None = None
        self._last_debug_monotonic = 0.0
        self._frames_received = 0
        self._decode_errors = 0
        self._last_error = ""
        self.get_logger().info("Color perception ready; motion output is absent")

    def _declare_parameters(self) -> None:
        defaults: dict[str, Any] = {
            "image_topic": "/csi/image_raw/compressed",
            "detections_topic": "/color_sorter/detections",
            "debug_topic": "/color_sorter/debug/compressed",
            "health_topic": "/color_sorter/perception_health",
            "image_timeout": 0.5,
            "debug_fps": 5.0,
            "debug_jpeg_quality": 65,
            "roi_top_ratio": 0.10,
            "morphology_kernel_size": 5,
            "minimum_area_ratio": 0.001,
            "maximum_area_ratio": 0.35,
            "minimum_aspect_ratio": 0.15,
            "maximum_aspect_ratio": 1.10,
            "minimum_extent": 0.40,
            "minimum_solidity": 0.75,
            "tracker_maximum_distance": 0.25,
            "tracker_maximum_missed": 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("colors", ["blue", "green", "red"])

        built_in_ranges = {
            "blue": [([88, 180, 60], [100, 255, 255])],
            "green": [([74, 70, 55], [85, 230, 135])],
            "red": [
                ([0, 205, 100], [6, 255, 190]),
                ([168, 205, 100], [179, 255, 190]),
                ([0, 219, 37], [8, 255, 109]),
                ([177, 219, 37], [179, 255, 109]),
            ],
        }
        colors = tuple(str(item) for item in self.get_parameter("colors").value)
        for color in colors:
            defaults_for_color = built_in_ranges.get(color, [])
            count_name = f"hsv.{color}.range_count"
            if defaults_for_color:
                self.declare_parameter(count_name, len(defaults_for_color))
            else:
                self.declare_parameter(count_name)
            parameter = self.get_parameter(count_name)
            if parameter.type_ == Parameter.Type.NOT_SET:
                raise ValueError(f"missing {count_name} for configured color")
            count = int(parameter.value)
            if count < 1:
                raise ValueError(f"{color} must have at least one HSV range")
            for index in range(1, count + 1):
                pair = (
                    defaults_for_color[index - 1]
                    if index <= len(defaults_for_color)
                    else None
                )
                for bound, default_index in (("lower", 0), ("upper", 1)):
                    name = f"hsv.{color}.{bound}_{index}"
                    if pair is None:
                        self.declare_parameter(name)
                    else:
                        self.declare_parameter(name, pair[default_index])
                    value = self.get_parameter(name)
                    if value.type_ == Parameter.Type.NOT_SET:
                        raise ValueError(f"missing {name} for configured color")

    def _load_ranges(self) -> dict[str, tuple[HsvRange, ...]]:
        result: dict[str, tuple[HsvRange, ...]] = {}
        colors = tuple(str(item) for item in self.get_parameter("colors").value)
        if len(colors) != len(set(colors)):
            raise ValueError("colors must be unique")
        for color in colors:
            count = self._int(f"hsv.{color}.range_count")
            if count < 1:
                raise ValueError(f"{color} must have an HSV range")
            items = []
            for index in range(1, count + 1):
                lower = self._int_tuple(f"hsv.{color}.lower_{index}")
                upper = self._int_tuple(f"hsv.{color}.upper_{index}")
                items.append(HsvRange(lower, upper))
            result[color] = tuple(items)
        return result

    def _on_image(self, message: CompressedImage) -> None:
        image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self._decode_errors += 1
            self._last_error = "JPEG decode failed"
            return
        try:
            result = self._detector.detect(image)
            tracked = self._tracker.update(result.detections)
        except Exception as error:
            self._last_error = str(error)
            self.get_logger().error(f"perception failed: {error}")
            return
        now = time.monotonic()
        self._last_image_monotonic = now
        self._frames_received += 1
        self._last_error = ""
        output = ColorObjectArray()
        output.header = message.header
        for item in tracked:
            detection = item.detection
            detected = ColorObject()
            detected.header = message.header
            detected.track_id = item.track_id
            detected.color = detection.color
            detected.shape = detection.shape
            detected.confidence = detection.confidence
            detected.roi.x_offset = detection.x
            detected.roi.y_offset = detection.y
            detected.roi.width = detection.width
            detected.roi.height = detection.height
            detected.center_x = detection.center_x
            detected.center_y = detection.center_y
            detected.normalized_x = detection.normalized_x
            detected.normalized_y = detection.normalized_y
            detected.normalized_width = detection.width / image.shape[1]
            detected.area_ratio = detection.area_ratio
            output.objects.append(detected)
        self._detections_publisher.publish(output)
        if (
            self._debug_publisher.get_subscription_count() > 0
            and now - self._last_debug_monotonic >= self._debug_period
        ):
            self._publish_debug(result.debug_image, message)
            self._last_debug_monotonic = now

    def _publish_debug(self, image: np.ndarray, source: CompressedImage) -> None:
        success, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self._debug_quality]
        )
        if success:
            message = CompressedImage()
            message.header = source.header
            message.format = "jpeg"
            message.data = encoded.tobytes()
            self._debug_publisher.publish(message)

    def _publish_health(self) -> None:
        now = time.monotonic()
        frame_age = (
            None
            if self._last_image_monotonic is None
            else now - self._last_image_monotonic
        )
        message = String()
        message.data = json.dumps(
            {
                "healthy": frame_age is not None and frame_age <= self._image_timeout,
                "frame_age": frame_age,
                "frames_received": self._frames_received,
                "decode_errors": self._decode_errors,
                "last_error": self._last_error,
            },
            separators=(",", ":"),
        )
        self._health_publisher.publish(message)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _int_tuple(self, name: str) -> tuple[int, int, int]:
        values = tuple(int(value) for value in self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly three values")
        return values


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ColorObjectDetectorNode | None = None
    try:
        node = ColorObjectDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
