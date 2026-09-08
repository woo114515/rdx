"""Capture operator-triggered camera/LiDAR correspondences without motion."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from color_object_sorter_interfaces.msg import ColorObjectArray, CylinderSnapshot
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .candidate_validation_node import _transform_xy


FIELDS = (
    "stamp_sec",
    "stamp_nanosec",
    "candidate_id",
    "track_id",
    "color",
    "local_x",
    "local_y",
    "bearing",
    "distance",
    "normalized_x",
)


class CameraLidarCaptureNode(Node):
    """Write one explicitly selected correspondence per service call."""

    def __init__(self) -> None:
        super().__init__("camera_lidar_calibration_capture")
        defaults = {
            "candidates_topic": "/cylinder_snapshot/candidates",
            "detections_topic": "/color_sorter/detections",
            "projection_frame": "lidar_link",
            "output_file": "camera_lidar_calibration.csv",
            "candidate_id": 0,
            "detection_track_id": 0,
            "expected_color": "",
            "maximum_data_age": 1.0,
            "transform_timeout": 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._snapshot: CylinderSnapshot | None = None
        self._detections: ColorObjectArray | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        snapshot_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # snapshot_builder publishes the live candidate set as VOLATILE.
            # Requesting TRANSIENT_LOCAL here is incompatible with that
            # publisher and prevents this callback from ever receiving data.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CylinderSnapshot,
            str(self.get_parameter("candidates_topic").value),
            self._on_snapshot,
            snapshot_qos,
        )
        self.create_subscription(
            ColorObjectArray,
            str(self.get_parameter("detections_topic").value),
            self._on_detections,
            10,
        )
        self.create_service(
            Trigger,
            "/camera_lidar_calibration/capture",
            self._capture,
        )
        self.get_logger().info(
            "Camera/LiDAR capture ready; motion output is absent"
        )

    def _on_snapshot(self, message: CylinderSnapshot) -> None:
        self._snapshot = message

    def _on_detections(self, message: ColorObjectArray) -> None:
        self._detections = message

    def _capture(self, request, response):
        del request
        snapshot = self._snapshot
        detections = self._detections
        if snapshot is None or detections is None:
            return _failure(response, "candidate or detection input is unavailable")
        now = self.get_clock().now()
        maximum_age = float(self.get_parameter("maximum_data_age").value)
        detection_age = (now - Time.from_msg(detections.header.stamp)).nanoseconds / 1e9
        snapshot_age = (now - Time.from_msg(snapshot.header.stamp)).nanoseconds / 1e9
        if max(detection_age, snapshot_age) > maximum_age:
            return _failure(response, "candidate or detection input is stale")

        candidate_id = int(self.get_parameter("candidate_id").value)
        candidates = list(snapshot.candidates)
        if candidate_id > 0:
            candidates = [item for item in candidates if item.candidate_id == candidate_id]
        if len(candidates) != 1:
            return _failure(
                response,
                "select exactly one candidate with the candidate_id parameter",
            )

        track_id = int(self.get_parameter("detection_track_id").value)
        expected_color = str(self.get_parameter("expected_color").value)
        objects = list(detections.objects)
        if track_id > 0:
            objects = [item for item in objects if item.track_id == track_id]
        if expected_color:
            objects = [item for item in objects if item.color == expected_color]
        if len(objects) != 1:
            return _failure(
                response,
                "select exactly one detection with detection_track_id/expected_color",
            )

        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("projection_frame").value),
                snapshot.header.frame_id,
                Time.from_msg(detections.header.stamp),
                timeout=Duration(
                    seconds=float(self.get_parameter("transform_timeout").value)
                ),
            )
        except TransformException as error:
            return _failure(response, f"candidate transform unavailable: {error}")
        candidate = candidates[0]
        detected = objects[0]
        local_x, local_y = _transform_xy(
            candidate.position.x,
            candidate.position.y,
            transform.transform,
        )
        distance = math.hypot(local_x, local_y)
        if local_x <= 0.0 or distance <= 0.0:
            return _failure(response, "selected candidate is not in front of the LiDAR")
        bearing = math.atan2(local_y, local_x)
        output = Path(str(self.get_parameter("output_file").value)).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        write_header = not output.exists() or output.stat().st_size == 0
        with output.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow(
                {
                    "stamp_sec": detections.header.stamp.sec,
                    "stamp_nanosec": detections.header.stamp.nanosec,
                    "candidate_id": candidate.candidate_id,
                    "track_id": detected.track_id,
                    "color": detected.color,
                    "local_x": f"{local_x:.12g}",
                    "local_y": f"{local_y:.12g}",
                    "bearing": f"{bearing:.12g}",
                    "distance": f"{distance:.12g}",
                    "normalized_x": f"{detected.normalized_x:.12g}",
                }
            )
        response.success = True
        response.message = (
            f"captured candidate={candidate.candidate_id} track={detected.track_id} "
            f"bearing={bearing:.4f} range={distance:.3f} "
            f"normalized_x={detected.normalized_x:.4f} file={output}"
        )
        return response


def _failure(response, message: str):
    response.success = False
    response.message = message
    return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraLidarCaptureNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
