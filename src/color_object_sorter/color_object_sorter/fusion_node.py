"""ROS node associating color detections with LiDAR ranges."""

from __future__ import annotations

import math

import rclpy
from color_object_sorter_interfaces.msg import (
    ColorObjectArray,
    LocalizedColorObject,
    LocalizedColorObjectArray,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .fusion import associate_scan, camera_bearing


class ColorLidarFusionNode(Node):
    """Publish range-bearing observations without producing motion commands."""

    def __init__(self) -> None:
        super().__init__("color_lidar_fusion")
        self.declare_parameter("detections_topic", "/color_sorter/detections")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("output_topic", "/color_sorter/localized_objects")
        self.declare_parameter("center_normalized_x", -0.069)
        self.declare_parameter("radians_per_normalized_x", -0.3926990817)
        self.declare_parameter("association_half_window", math.radians(4.0))
        self.declare_parameter("maximum_scan_age", 0.25)
        self.declare_parameter("maximum_range_jump", 0.15)
        self.declare_parameter("minimum_cluster_points", 2)
        self._scan: LaserScan | None = None
        self._publisher = self.create_publisher(
            LocalizedColorObjectArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ColorObjectArray,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )
        self.get_logger().info("Color/LiDAR fusion ready; motion output is absent")

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = message

    def _on_detections(self, message: ColorObjectArray) -> None:
        output = LocalizedColorObjectArray()
        scan = self._scan
        scan_is_fresh = scan is not None and abs(
            self._stamp_seconds(message.header.stamp)
            - self._stamp_seconds(scan.header.stamp)
        ) <= float(self.get_parameter("maximum_scan_age").value)
        output.header = scan.header if scan_is_fresh and scan is not None else message.header

        for detected in message.objects:
            item = LocalizedColorObject()
            item.header = output.header
            item.track_id = detected.track_id
            item.color = detected.color
            item.shape = detected.shape
            item.confidence = detected.confidence
            item.normalized_x = detected.normalized_x
            predicted = camera_bearing(
                detected.normalized_x,
                float(self.get_parameter("center_normalized_x").value),
                float(self.get_parameter("radians_per_normalized_x").value),
            )
            item.bearing = predicted
            item.distance = math.nan
            item.matched = False
            if scan_is_fresh and scan is not None:
                match = associate_scan(
                    scan.ranges,
                    scan.angle_min,
                    scan.angle_increment,
                    predicted,
                    float(self.get_parameter("association_half_window").value),
                    scan.range_min,
                    scan.range_max,
                    float(self.get_parameter("maximum_range_jump").value),
                    int(self.get_parameter("minimum_cluster_points").value),
                )
                if match is not None:
                    item.bearing = match.bearing
                    item.distance = match.distance
                    item.matched = True
            output.objects.append(item)
        self._publisher.publish(output)

    @staticmethod
    def _stamp_seconds(stamp: object) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ColorLidarFusionNode | None = None
    try:
        node = ColorLidarFusionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
