"""ROS adapter for motion-free first-target selection visualization."""

from __future__ import annotations

import json
import math

import rclpy
from color_object_sorter_interfaces.msg import ValidatedCylinderArray
from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .geometry import (
    Target,
    build_push_preview,
    destination_slot,
    local_offset_to_map,
    path_length,
    select_right_first,
)


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
        self.declare_parameter("robot_center_clearance", 0.21)
        self.declare_parameter("contact_offset", 0.22)
        self.declare_parameter("release_retreat", 0.15)
        self.declare_parameter("envelope_exit_clearance", 0.10)
        self.declare_parameter("delivered_exclusion_radius", 0.18)
        self.declare_parameter("destination_slot_spacing", 0.18)
        self.declare_parameter("transform_timeout", 0.25)
        self.declare_parameter(
            "destination_colors", ["blue", "green", "red"]
        )
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
        self._snapshot: ValidatedCylinderArray | None = None
        self._initial_color_counts: dict[str, int] = {}
        self._task_home: tuple[float, float, float] | None = None
        self._task_field_center: tuple[float, float] | None = None
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
        self._target_path = self.create_publisher(
            Path, "/cylinder_push_plan/target_path", latched
        )
        self._approach_path = self.create_publisher(
            Path, "/cylinder_push_plan/approach_path", latched
        )
        self._contact_path = self.create_publisher(
            Path, "/cylinder_push_plan/contact_path", latched
        )
        self._robot_push_path = self.create_publisher(
            Path, "/cylinder_push_plan/robot_push_path", latched
        )
        self._return_path = self.create_publisher(
            Path, "/cylinder_push_plan/return_path", latched
        )
        self._keepout = self.create_publisher(
            PolygonStamped, "/cylinder_push_plan/remaining_keepout", latched
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
        self.create_service(
            Trigger, "/cylinder_push_plan/reset_task", self._reset_task
        )
        self.get_logger().info("Selection planner ready; motion output is absent")

    def _on_snapshot(self, message: ValidatedCylinderArray) -> None:
        self._snapshot = message
        for color, count in zip(
            message.expected_colors, message.expected_color_counts
        ):
            name = str(color)
            self._initial_color_counts[name] = max(
                self._initial_color_counts.get(name, 0), int(count)
            )

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
        rotation = transform.transform.rotation
        robot_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y**2 + rotation.z**2),
        )
        current_pose = (translation.x, translation.y, robot_yaw)
        if self._task_home is None:
            self._task_home = current_pose
        task_home = self._task_home
        try:
            selection = select_right_first(
                targets,
                translation.x,
                translation.y,
                self._float("staging_clearance"),
            )
            if self._task_field_center is None:
                self._task_field_center = (
                    selection.envelope_x,
                    selection.envelope_y,
                )
            task_field_center = self._task_field_center
            destination, slot_index, slot_count = self._destination_for_color(
                selection.target.color,
                task_field_center[0],
                task_field_center[1],
                task_home[2],
            )
            plan = build_push_preview(
                targets,
                translation.x,
                translation.y,
                robot_yaw,
                destination[0],
                destination[1],
                self._float("staging_clearance"),
                self._float("robot_center_clearance"),
                self._float("contact_offset"),
                self._float("release_retreat"),
                self._float("envelope_exit_clearance"),
                selection,
                self._float("delivered_exclusion_radius"),
                task_home,
            )
        except ValueError as error:
            return self._failure(response, str(error))
        stamp = self.get_clock().now().to_msg()
        pose = _pose(fixed_frame, stamp, plan.staging_x, plan.staging_y, plan.staging_yaw)
        self._staging.publish(pose)
        self._approach_path.publish(_path(fixed_frame, stamp, plan.approach_path))
        contact_path = (
            (plan.staging_x, plan.staging_y),
            plan.robot_push_path[0],
        )
        self._contact_path.publish(_path(fixed_frame, stamp, contact_path))
        self._target_path.publish(_path(fixed_frame, stamp, plan.target_path))
        self._robot_push_path.publish(
            _path(fixed_frame, stamp, plan.robot_push_path)
        )
        self._return_path.publish(
            _path(
                fixed_frame,
                stamp,
                plan.return_path,
                final_yaw=plan.home_yaw,
            )
        )
        self._keepout.publish(
            _polygon(fixed_frame, stamp, plan.keepout.boundary)
        )
        self._markers.publish(
            _plan_markers(
                plan,
                fixed_frame,
                stamp,
                self._float("delivered_exclusion_radius"),
            )
        )
        status = {
            "ready": True,
            "motion_output": False,
            "plan_stamp": [stamp.sec, stamp.nanosec],
            "map_collision_checked": False,
            "target_id": plan.selection.target.candidate_id,
            "target_color": plan.selection.target.color,
            "initial_envelope_radius": round(plan.selection.envelope_radius, 4),
            "remaining_count": len(targets) - 1,
            "keepout_clearance": plan.keepout.clearance,
            "destination": [round(plan.destination_x, 4), round(plan.destination_y, 4)],
            "delivered_exclusion_radius": self._float(
                "delivered_exclusion_radius"
            ),
            "destination_slot_index": slot_index,
            "destination_slot_count": slot_count,
            "task_home": [round(value, 4) for value in task_home],
            "task_field_center": [
                round(value, 4) for value in task_field_center
            ],
            "approach_path_length": round(path_length(plan.approach_path), 4),
            "contact_path_length": round(path_length(contact_path), 4),
            "push_path_length": round(path_length(plan.robot_push_path), 4),
            "return_path_length": round(path_length(plan.return_path), 4),
        }
        self._status.publish(String(data=json.dumps(status, sort_keys=True)))
        response.success = True
        response.message = (
            f"previewed candidate {plan.selection.target.candidate_id} "
            f"({plan.selection.target.color}); motion disabled"
        )
        return response

    def _destination_for_color(self, color, field_x, field_y, task_yaw):
        colors = tuple(str(item) for item in self.get_parameter("destination_colors").value)
        if color not in colors:
            raise ValueError(f"no destination configured for color {color!r}")
        local_x = self._float(f"destinations.{color}.x")
        local_y = self._float(f"destinations.{color}.y")
        assert self._snapshot is not None
        current_counts = dict(
            zip(
                (str(item) for item in self._snapshot.expected_colors),
                (int(item) for item in self._snapshot.expected_color_counts),
            )
        )
        if color not in current_counts or current_counts[color] < 1:
            raise ValueError(f"selected color {color!r} is absent from inventory")
        slot_count = self._initial_color_counts.get(color, current_counts[color])
        slot_index = slot_count - current_counts[color]
        local_x, local_y = destination_slot(
            local_x,
            local_y,
            slot_index,
            slot_count,
            self._float("destination_slot_spacing"),
        )
        return (
            local_offset_to_map(
                field_x,
                field_y,
                task_yaw,
                local_x,
                local_y,
            ),
            slot_index,
            slot_count,
        )

    def _float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _reset(self, request, response):
        del request
        self._clear_preview()
        response.success = True
        response.message = "push selection reset; task anchor preserved"
        return response

    def _reset_task(self, request, response):
        del request
        self._clear_preview()
        self._task_home = None
        self._task_field_center = None
        self._initial_color_counts.clear()
        response.success = True
        response.message = "push task anchor and slot history reset"
        return response

    def _clear_preview(self) -> None:
        self._snapshot = None
        clear = Marker()
        clear.action = Marker.DELETEALL
        self._markers.publish(MarkerArray(markers=[clear]))
        empty = Path()
        self._approach_path.publish(empty)
        self._contact_path.publish(empty)
        self._target_path.publish(empty)
        self._robot_push_path.publish(empty)
        self._return_path.publish(empty)
        self._keepout.publish(PolygonStamped())
        self._status.publish(String(data='{"ready": false, "reason": "reset"}'))

    def _failure(self, response, reason: str):
        self._status.publish(
            String(data=json.dumps({"ready": False, "reason": reason}, sort_keys=True))
        )
        response.success = False
        response.message = reason
        return response


def _plan_markers(
    plan, frame_id: str, stamp, delivered_exclusion_radius: float
) -> MarkerArray:
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
                x=plan.selection.envelope_x
                + plan.selection.envelope_radius * math.cos(angle),
                y=plan.selection.envelope_y
                + plan.selection.envelope_radius * math.sin(angle),
            )
        )
    target = Marker()
    target.header = envelope.header
    target.ns = "selected_target"
    target.id = 2
    target.type = Marker.SPHERE
    target.action = Marker.ADD
    target.pose.position.x = plan.selection.target.x
    target.pose.position.y = plan.selection.target.y
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
    keepout = _line_marker(
        envelope.header,
        "remaining_keepout",
        4,
        plan.keepout.boundary,
        (1.0, 0.0, 0.0, 0.95),
        closed=True,
        width=0.035,
    )
    hull = _line_marker(
        envelope.header,
        "remaining_hull",
        5,
        plan.keepout.hull,
        (0.7, 0.0, 0.7, 0.8),
        closed=True,
    )
    target_path = _line_marker(
        envelope.header,
        "target_push_path",
        6,
        plan.target_path,
        (0.1, 1.0, 0.1, 1.0),
        width=0.035,
    )
    approach_path = _line_marker(
        envelope.header,
        "approach_path",
        11,
        plan.approach_path,
        (1.0, 0.85, 0.0, 0.9),
    )
    contact_path = _line_marker(
        envelope.header,
        "contact_path",
        12,
        ((plan.staging_x, plan.staging_y), plan.robot_push_path[0]),
        (1.0, 0.4, 0.0, 1.0),
        width=0.03,
    )
    robot_path = _line_marker(
        envelope.header,
        "robot_push_path",
        7,
        plan.robot_push_path,
        (0.0, 0.8, 1.0, 0.9),
    )
    return_path = _line_marker(
        envelope.header,
        "return_path",
        8,
        plan.return_path,
        (1.0, 1.0, 1.0, 0.9),
    )
    destination = Marker()
    destination.header = envelope.header
    destination.ns = "destination"
    destination.id = 9
    destination.type = Marker.CYLINDER
    destination.action = Marker.ADD
    destination.pose.position.x = plan.destination_x
    destination.pose.position.y = plan.destination_y
    destination.pose.orientation.w = 1.0
    destination.scale.x = destination.scale.y = 2.0 * delivered_exclusion_radius
    destination.scale.z = 0.02
    destination.color.g = 1.0
    destination.color.a = 0.45
    home = Marker()
    home.header = envelope.header
    home.ns = "home_pose"
    home.id = 10
    home.type = Marker.ARROW
    home.action = Marker.ADD
    home.pose.position.x = plan.home_x
    home.pose.position.y = plan.home_y
    home.pose.orientation.z = math.sin(plan.home_yaw / 2.0)
    home.pose.orientation.w = math.cos(plan.home_yaw / 2.0)
    home.scale.x = 0.25
    home.scale.y = home.scale.z = 0.05
    home.color.r = home.color.g = home.color.b = 1.0
    home.color.a = 1.0
    return MarkerArray(
        markers=[
            clear,
            envelope,
            target,
            staging,
            keepout,
            hull,
            approach_path,
            contact_path,
            target_path,
            robot_path,
            return_path,
            destination,
            home,
        ]
    )


def _line_marker(header, namespace, marker_id, points, color, closed=False, width=0.02):
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = width
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    output = list(points)
    if closed and len(output) > 1:
        output.append(output[0])
    marker.points = [Point(x=x, y=y, z=0.015) for x, y in output]
    return marker


def _pose(frame_id, stamp, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def _path(frame_id, stamp, points, final_yaw=None):
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = stamp
    for index, (x, y) in enumerate(points):
        if index + 1 < len(points):
            following = points[index + 1]
            yaw = math.atan2(following[1] - y, following[0] - x)
        elif final_yaw is not None:
            yaw = final_yaw
        elif index:
            previous = points[index - 1]
            yaw = math.atan2(y - previous[1], x - previous[0])
        else:
            yaw = 0.0
        path.poses.append(_pose(frame_id, stamp, x, y, yaw))
    return path


def _polygon(frame_id, stamp, points):
    polygon = PolygonStamped()
    polygon.header.frame_id = frame_id
    polygon.header.stamp = stamp
    polygon.polygon.points = [Point32(x=x, y=y, z=0.0) for x, y in points]
    return polygon


def main() -> None:
    rclpy.init()
    node = SelectionPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
