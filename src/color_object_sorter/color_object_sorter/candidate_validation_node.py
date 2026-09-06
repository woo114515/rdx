"""ROS adapter for multi-view visual validation of LiDAR candidates."""

from __future__ import annotations

from collections import Counter
import math

import rclpy
from color_object_sorter_interfaces.msg import (
    ColorObjectArray,
    CylinderSnapshot,
    ValidatedCylinder,
    ValidatedCylinderArray,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .candidate_validation import (
    CandidateColorValidator,
    FrameEvidence,
    associate_visual_detections,
    inventory_matches,
    visual_association_window,
)


class CandidateValidationNode(Node):
    """Assign color evidence to spatial candidates without motion output."""

    def __init__(self) -> None:
        super().__init__("cylinder_candidate_validator")
        defaults = {
            "candidates_topic": "/cylinder_snapshot/candidates",
            "detections_topic": "/color_sorter/detections",
            "output_topic": "/cylinder_snapshot/validated_objects",
            "marker_topic": "/cylinder_snapshot/validation_markers",
            "projection_frame": "lidar_link",
            "center_normalized_x": -0.069,
            "radians_per_normalized_x": -0.3926990817,
            "minimum_normalized_window": 0.08,
            "normalized_window_padding": 0.03,
            "edge_window_start": 0.75,
            "maximum_edge_normalized_window": 0.14,
            "visual_ambiguity_margin": 0.04,
            "edge_margin": 0.08,
            "minimum_color_observations": 5,
            "minimum_visible_observations": 8,
            "minimum_color_confidence": 0.65,
            "auto_lock_when_ready": True,
            "transform_timeout": 0.15,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._validator = CandidateColorValidator(
            self._int("minimum_color_observations"),
            self._int("minimum_visible_observations"),
            self._float("minimum_color_confidence"),
        )
        self._snapshot: CylinderSnapshot | None = None
        self._locked = False
        self._last_output: ValidatedCylinderArray | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            ValidatedCylinderArray, self._string("output_topic"), latched
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, self._string("marker_topic"), latched
        )
        self.create_subscription(
            CylinderSnapshot,
            self._string("candidates_topic"),
            self._on_snapshot,
            10,
        )
        self.create_subscription(
            ColorObjectArray,
            self._string("detections_topic"),
            self._on_detections,
            10,
        )
        self.create_service(Trigger, "/cylinder_validation/lock", self._lock)
        self.create_service(Trigger, "/cylinder_validation/reset", self._reset)
        self.get_logger().info(
            "Candidate visual validation ready; motion output is absent"
        )

    def _on_snapshot(self, message: CylinderSnapshot) -> None:
        if not self._locked:
            self._snapshot = message

    def _on_detections(self, message: ColorObjectArray) -> None:
        if self._locked or self._snapshot is None:
            return
        snapshot = self._snapshot
        if not snapshot.header.frame_id:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._string("projection_frame"),
                snapshot.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._float("transform_timeout")),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot project candidates into camera: {error}",
                throttle_duration_sec=2.0,
            )
            return

        allowed_colors = tuple(snapshot.expected_colors)
        if not allowed_colors:
            allowed_colors = tuple(sorted({item.color for item in message.objects}))
        projections = {}
        for candidate in snapshot.candidates:
            local_x, local_y = _transform_xy(
                candidate.position.x,
                candidate.position.y,
                transform.transform,
            )
            visibility, normalized_x = self._visibility(local_x, local_y)
            projections[candidate.candidate_id] = (visibility, normalized_x)

        visible_candidates = [
            item
            for item in snapshot.candidates
            if projections[item.candidate_id][0] in ("visible", "edge")
        ]
        detections = [
            item
            for item in message.objects
            if not allowed_colors or item.color in allowed_colors
        ]
        detection_windows = [
            visual_association_window(
                item.normalized_x,
                item.normalized_width,
                self._float("minimum_normalized_window"),
                self._float("normalized_window_padding"),
                self._float("edge_window_start"),
                self._float("maximum_edge_normalized_window"),
            )
            for item in detections
        ]
        associations = associate_visual_detections(
            [projections[item.candidate_id][1] for item in visible_candidates],
            [item.normalized_x for item in detections],
            detection_windows,
            self._float("visual_ambiguity_margin"),
        )
        association_by_id = {
            candidate.candidate_id: association
            for candidate, association in zip(visible_candidates, associations)
        }
        evidence = []
        for candidate in snapshot.candidates:
            visibility, projected_x = projections[candidate.candidate_id]
            association = association_by_id.get(candidate.candidate_id)
            if association is None:
                evidence.append(FrameEvidence(candidate.candidate_id, visibility, {}))
                continue
            if association.status == "ambiguous":
                evidence.append(FrameEvidence(candidate.candidate_id, "ambiguous", {}))
                continue
            if association.detection_index is None:
                evidence.append(FrameEvidence(candidate.candidate_id, visibility, {}))
                continue
            detected = detections[association.detection_index]
            window = detection_windows[association.detection_index]
            error = abs(detected.normalized_x - projected_x)
            score = float(detected.confidence) * (1.0 - 0.5 * error / window)
            evidence.append(
                FrameEvidence(candidate.candidate_id, visibility, {detected.color: score})
            )

        active_ids = {item.candidate_id for item in snapshot.candidates}
        validations = self._validator.update(evidence, active_ids, allowed_colors)
        by_id = {item.candidate_id: item for item in validations}
        output = ValidatedCylinderArray()
        output.header = message.header
        output.header.frame_id = snapshot.header.frame_id
        output.expected_colors = list(snapshot.expected_colors)
        output.expected_color_counts = list(snapshot.expected_color_counts)
        output.ready = inventory_matches(
            validations,
            snapshot.expected_colors,
            snapshot.expected_color_counts,
        )
        states = Counter(item.state for item in validations)
        output.status = ";".join(
            f"{name}:{states.get(name, 0)}"
            for name in ("validated", "uncertain", "unobserved", "rejected")
        )
        output.locked = False
        for candidate in snapshot.candidates:
            validation = by_id[candidate.candidate_id]
            item = ValidatedCylinder()
            item.header = output.header
            item.candidate_id = candidate.candidate_id
            item.position = candidate.position
            item.radius = candidate.radius
            item.position_confidence = candidate.confidence
            item.color = validation.color
            item.color_confidence = validation.color_confidence
            item.color_names = [name for name, _score in validation.color_scores]
            item.color_scores = [score for _name, score in validation.color_scores]
            item.visible_observations = validation.visible_observations
            item.color_observations = validation.color_observations
            item.state = validation.state
            output.objects.append(item)
        if output.ready and bool(self.get_parameter("auto_lock_when_ready").value):
            output.locked = True
            self._locked = True
            self.get_logger().info(
                "Validated inventory complete; snapshot locked automatically"
            )
        self._last_output = output
        self._publisher.publish(output)
        self._publish_markers(output)

    def _visibility(self, x: float, y: float) -> tuple[str, float]:
        if x <= 0.0:
            return "unobserved", math.nan
        bearing = math.atan2(y, x)
        scale = self._float("radians_per_normalized_x")
        normalized_x = bearing / scale + self._float("center_normalized_x")
        if not -1.0 <= normalized_x <= 1.0:
            return "unobserved", normalized_x
        edge = self._float("edge_margin")
        if abs(normalized_x) > 1.0 - edge:
            return "edge", normalized_x
        return "visible", normalized_x

    def _lock(self, request, response):
        del request
        if self._last_output is None or not self._last_output.ready:
            response.success = False
            response.message = "validated inventory is not ready"
            return response
        self._locked = True
        self._last_output.locked = True
        self._publisher.publish(self._last_output)
        self._publish_markers(self._last_output)
        response.success = True
        response.message = "validated snapshot locked"
        return response

    def _reset(self, request, response):
        del request
        self._validator.clear()
        self._snapshot = None
        self._last_output = None
        self._locked = False
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self._marker_publisher.publish(markers)
        response.success = True
        response.message = "candidate validation reset"
        return response

    def _publish_markers(self, output: ValidatedCylinderArray) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for item in output.objects:
            marker = Marker()
            marker.header = output.header
            marker.ns = "validated_candidates"
            marker.id = int(item.candidate_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = item.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = 2.0 * item.radius
            marker.scale.z = 0.25
            marker.color.r, marker.color.g, marker.color.b = _marker_color(
                item.state, item.color
            )
            marker.color.a = 0.9 if item.state != "rejected" else 0.25
            markers.markers.append(marker)
        self._marker_publisher.publish(markers)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _int(self, name: str) -> int:
        return int(self.get_parameter(name).value)


def _transform_xy(x: float, y: float, transform) -> tuple[float, float]:
    quaternion = transform.rotation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        transform.translation.x + cosine * x - sine * y,
        transform.translation.y + sine * x + cosine * y,
    )


def _marker_color(state: str, color: str) -> tuple[float, float, float]:
    if state == "validated":
        return {
            "blue": (0.1, 0.3, 1.0),
            "green": (0.1, 1.0, 0.2),
            "red": (1.0, 0.1, 0.1),
        }.get(color, (0.8, 0.2, 1.0))
    if state == "rejected":
        return 0.4, 0.4, 0.4
    if state == "unobserved":
        return 0.7, 0.7, 0.7
    return 1.0, 0.8, 0.1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CandidateValidationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
