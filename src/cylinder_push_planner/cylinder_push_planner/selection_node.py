"""ROS adapter for motion-free first-target selection visualization."""

from __future__ import annotations

import json
import math

import rclpy
from color_object_sorter_interfaces.msg import ValidatedCylinderArray
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import Target, select_right_first


class SelectionPlannerNode(Node):
    """Generate a target and staging pose without publishing motion commands."""

    def __init__(self) -> None:
        super().__init__("cylinder_push_selection_planner")
        self.declare_parameter(
            "input_topic", "/cylinder_snapshot/validated_objects"
        )
        self.declare_parameter("fixed_frame", "map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("staging_clearance", 0.25)
        self.declare_parameter("transform_timeout", 0.25)
        self._snapshot: ValidatedCylinderArray | None = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._markers = self.create_publisher(
            MarkerArray, "/cylinder_push_plan/markers", latched
        )
        self._staging = self.create_publisher(
            PoseStamped, "/cylinder_push_plan/staging_pose", latched
        )
        self._status = self.create_publisher(
            String, "/cylinder_push_plan/status", latched
        )
        self.create_subscription(
            ValidatedCylinderArray,
            str(self.get_parameter("input_topic").value),
            self._on_snapshot,
            latched,
        )
        self.create_service(Trigger, "/cylinder_push_plan/generate", self._generate)
        self.create_service(Trigger, "/cylinder_push_plan/reset", self._reset)
        self.get_logger().info("Selection planner ready; motion output is absent")

    def _on_snapshot(self, message: ValidatedCylinderArray) -> None:
        self._snapshot = message

    def _generate(self, request, response):
        del request
        if self._snapshot is None:
            return self._failure(response, "no validated snapshot received")
        if not self._snapshot.ready:
            return self._failure(response, "validated inventory is not ready")
        if not self._snapshot.locked:
            return self._failure(response, "validated snapshot is not locked")
        fixed_frame = str(self.get_parameter("fixed_frame").value)
        try:
            transform = self._tf_buffer.lookup_transform(
                fixed_frame,
                str(self.get_parameter("robot_frame").value),
                Time(),
                timeout=Duration(
                    seconds=float(self.get_parameter("transform_timeout").value)
                ),
            )
        except TransformException as error:
            return self._failure(response, f"robot pose unavailable: {error}")
        targets = tuple(
            Target(
                candidate_id=item.candidate_id,
                x=item.position.x,
                y=item.position.y,
                color=item.color,
                radius=item.radius,
            )
            for item in self._snapshot.objects
            if item.state == "validated"
        )
        translation = transform.transform.translation
        try:
            plan = select_right_first(
                targets,
                translation.x,
                translation.y,
                float(self.get_parameter("staging_clearance").value),
            )
        except ValueError as error:
            return self._failure(response, str(error))
        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.frame_id = fixed_frame
        pose.header.stamp = stamp
        pose.pose.position.x = plan.staging_x
        pose.pose.position.y = plan.staging_y
        pose.pose.orientation.z = math.sin(plan.staging_yaw / 2.0)
        pose.pose.orientation.w = math.cos(plan.staging_yaw / 2.0)
        self._staging.publish(pose)
        self._markers.publish(_plan_markers(plan, fixed_frame, stamp))
        status = {
            "ready": True,
            "motion_output": False,
            "target_id": plan.target.candidate_id,
            "target_color": plan.target.color,
            "envelope_radius": round(plan.envelope_radius, 4),
        }
        self._status.publish(String(data=json.dumps(status, sort_keys=True)))
        response.success = True
        response.message = (
            f"selected candidate {plan.target.candidate_id} ({plan.target.color})"
        )
        return response

    def _reset(self, request, response):
        del request
        self._snapshot = None
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._markers.publish(MarkerArray(markers=[clear]))
        self._status.publish(String(data='{"ready": false, "reason": "reset"}'))
        response.success = True
        response.message = "push selection reset"
        return response

    def _failure(self, response, reason: str):
        self._status.publish(
            String(data=json.dumps({"ready": False, "reason": reason}, sort_keys=True))
        )
        response.success = False
        response.message = reason
        return response


def _plan_markers(plan, frame_id: str, stamp) -> MarkerArray:
    clear = Marker()
    clear.action = Marker.DELETEALL
    envelope = Marker()
    envelope.header.frame_id = frame_id
    envelope.header.stamp = stamp
    envelope.ns = "initial_envelope"
    envelope.id = 1
    envelope.type = Marker.LINE_STRIP
    envelope.action = Marker.ADD
    envelope.scale.x = 0.02
    envelope.color.r = 1.0
    envelope.color.g = 0.65
    envelope.color.a = 0.9
    for index in range(65):
        angle = 2.0 * math.pi * index / 64.0
        envelope.points.append(
            Point(
                x=plan.envelope_x + plan.envelope_radius * math.cos(angle),
                y=plan.envelope_y + plan.envelope_radius * math.sin(angle),
            )
        )
    target = Marker()
    target.header = envelope.header
    target.ns = "selected_target"
    target.id = 2
    target.type = Marker.SPHERE
    target.action = Marker.ADD
    target.pose.position.x = plan.target.x
    target.pose.position.y = plan.target.y
    target.pose.orientation.w = 1.0
    target.scale.x = target.scale.y = target.scale.z = 0.12
    target.color.r = 1.0
    target.color.a = 1.0
    staging = Marker()
    staging.header = envelope.header
    staging.ns = "staging_pose"
    staging.id = 3
    staging.type = Marker.ARROW
    staging.action = Marker.ADD
    staging.pose.position.x = plan.staging_x
    staging.pose.position.y = plan.staging_y
    staging.pose.orientation.z = math.sin(plan.staging_yaw / 2.0)
    staging.pose.orientation.w = math.cos(plan.staging_yaw / 2.0)
    staging.scale.x = 0.30
    staging.scale.y = 0.06
    staging.scale.z = 0.06
    staging.color.b = 1.0
    staging.color.a = 1.0
    return MarkerArray(markers=[clear, envelope, target, staging])


def main() -> None:
    rclpy.init()
    node = SelectionPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
