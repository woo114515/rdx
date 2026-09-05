"""ROS adapter for multi-frame object confirmation."""

from __future__ import annotations

import math

import rclpy
from color_object_sorter_interfaces.msg import (
    ConfirmedColorObject,
    ConfirmedColorObjectArray,
    LocalizedColorObjectArray,
)
from rclpy.node import Node

from .confirmation import Observation, TemporalConfirmer


class TemporalConfirmationNode(Node):
    """Publish only stable, repeatedly observed localized objects."""

    def __init__(self) -> None:
        super().__init__("temporal_object_confirmation")
        defaults = {
            "input_topic": "/color_sorter/localized_objects",
            "output_topic": "/color_sorter/confirmed_objects",
            "status_topic": "/color_sorter/confirmation_status",
            "window_seconds": 1.0,
            "minimum_observations": 6,
            "minimum_detection_rate": 0.6,
            "minimum_color_consistency": 0.8,
            "minimum_lidar_match_rate": 0.5,
            "maximum_bearing_stddev": math.radians(3.0),
            "maximum_distance_stddev": 0.10,
            "association_bearing": math.radians(6.0),
            "association_distance": 0.30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._confirmer = TemporalConfirmer(
            window_seconds=self._float("window_seconds"),
            minimum_observations=self._int("minimum_observations"),
            minimum_detection_rate=self._float("minimum_detection_rate"),
            minimum_color_consistency=self._float("minimum_color_consistency"),
            minimum_lidar_match_rate=self._float("minimum_lidar_match_rate"),
            maximum_bearing_stddev=self._float("maximum_bearing_stddev"),
            maximum_distance_stddev=self._float("maximum_distance_stddev"),
            association_bearing=self._float("association_bearing"),
            association_distance=self._float("association_distance"),
        )
        self._publisher = self.create_publisher(
            ConfirmedColorObjectArray,
            str(self.get_parameter("output_topic").value),
            10,
        )
        self._status_publisher = self.create_publisher(
            ConfirmedColorObjectArray,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self.create_subscription(
            LocalizedColorObjectArray,
            str(self.get_parameter("input_topic").value),
            self._on_objects,
            10,
        )
        self.get_logger().info("Temporal confirmation ready; motion output is absent")

    def _on_objects(self, message: LocalizedColorObjectArray) -> None:
        stamp = float(message.header.stamp.sec) + message.header.stamp.nanosec * 1e-9
        observations = [
            Observation(
                frame=0,
                stamp=stamp,
                color=item.color,
                shape=item.shape,
                confidence=item.confidence,
                bearing=item.bearing,
                distance=item.distance,
                matched=item.matched,
            )
            for item in message.objects
        ]
        summaries = self._confirmer.update(stamp, observations)
        output = ConfirmedColorObjectArray()
        output.header = message.header
        status = ConfirmedColorObjectArray()
        status.header = message.header
        for summary in summaries:
            item = ConfirmedColorObject()
            item.header = message.header
            item.track_id = summary.track_id
            item.color = summary.color
            item.shape = summary.shape
            item.confidence = summary.confidence
            item.bearing = summary.bearing
            item.distance = summary.distance
            item.observation_count = summary.observation_count
            item.detection_rate = summary.detection_rate
            item.color_consistency = summary.color_consistency
            item.lidar_match_rate = summary.lidar_match_rate
            item.bearing_stddev = summary.bearing_stddev
            item.distance_stddev = summary.distance_stddev
            item.confirmed = summary.confirmed
            status.objects.append(item)
            if summary.confirmed:
                output.objects.append(item)
        self._status_publisher.publish(status)
        self._publisher.publish(output)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: TemporalConfirmationNode | None = None
    try:
        node = TemporalConfirmationNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
