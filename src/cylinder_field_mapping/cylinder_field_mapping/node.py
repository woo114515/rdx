"""ROS node that builds persistent map-frame cylinder landmarks."""

from __future__ import annotations

import math

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from color_object_sorter_interfaces.msg import (
    ConfirmedColorObjectArray,
    CylinderField,
    MappedCylinder,
)
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from cylinder_field_mapping.field_map import (
    Circle,
    Cylinder,
    CylinderFieldMap,
    Observation,
)


class CylinderFieldMapperNode(Node):
    """Fuse confirmed polar observations into stable map-frame landmarks."""

    def __init__(self) -> None:
        super().__init__("cylinder_field_mapper")
        self.declare_parameter("input_topic", "/color_sorter/confirmation_status")
        self.declare_parameter("output_topic", "/cylinder_field/objects")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("association_distance", 0.15)
        self.declare_parameter("history_size", 25)
        self.declare_parameter("maximum_per_color", 2)
        self.declare_parameter("occluded_after", 0.75)
        self.declare_parameter("stale_after", 10.0)
        self.declare_parameter("minimum_observations", 3)
        self.declare_parameter("minimum_lock_objects", 6)
        self.declare_parameter("cylinder_radius", 0.035)
        self.declare_parameter("transform_timeout", 0.15)

        association_distance = float(
            self.get_parameter("association_distance").value
        )
        history_size = int(self.get_parameter("history_size").value)
        self._minimum_observations = int(
            self.get_parameter("minimum_observations").value
        )
        self._minimum_lock_objects = int(
            self.get_parameter("minimum_lock_objects").value
        )
        self._cylinder_radius = float(self.get_parameter("cylinder_radius").value)
        self._transform_timeout = float(
            self.get_parameter("transform_timeout").value
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._occluded_after = float(
            self.get_parameter("occluded_after").value
        )
        self._stale_after = float(self.get_parameter("stale_after").value)
        self._field = CylinderFieldMap(
            association_distance=association_distance,
            history_size=history_size,
            maximum_per_color=int(
                self.get_parameter("maximum_per_color").value
            ),
        )
        self._locked_envelope: Circle | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            CylinderField, str(self.get_parameter("output_topic").value), 10
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, "/cylinder_field/markers", 10
        )
        self.create_subscription(
            ConfirmedColorObjectArray,
            str(self.get_parameter("input_topic").value),
            self._on_objects,
            10,
        )
        self.create_service(
            Trigger, "/cylinder_field/lock_initial_envelope", self._lock_envelope
        )
        self.create_service(Trigger, "/cylinder_field/reset", self._reset)
        self.get_logger().info("Building map-frame cylinder field; motion disabled")

    def _on_objects(self, message: ConfirmedColorObjectArray) -> None:
        source_frame = message.header.frame_id
        if not source_frame:
            self.get_logger().warning("Ignoring observations without a frame_id")
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self._transform_timeout),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot transform cylinder observations: {error}",
                throttle_duration_sec=2.0,
            )
            return

        stamp = Time.from_msg(message.header.stamp).nanoseconds / 1e9
        observations = []
        for item in message.objects:
            if not item.confirmed or item.state != "confirmed":
                continue
            if not math.isfinite(item.distance) or item.distance <= 0.0:
                continue
            local_x = item.distance * math.cos(item.bearing)
            local_y = item.distance * math.sin(item.bearing)
            map_x, map_y = _transform_xy(local_x, local_y, transform.transform)
            observations.append(
                Observation(map_x, map_y, item.color, item.confidence, stamp)
            )
        self._field.update(observations, stamp=stamp)
        cylinders = self._field.active(
            self._occluded_after,
            self._stale_after,
        )
        self._publish(message.header.stamp, cylinders)

    def _publish(
        self,
        stamp: TimeMessage,
        cylinders: tuple[Cylinder, ...],
    ) -> None:
        message = CylinderField()
        message.header.stamp = stamp
        message.header.frame_id = self._map_frame
        for item in cylinders:
            mapped = MappedCylinder()
            mapped.header = message.header
            mapped.cylinder_id = item.cylinder_id
            mapped.color = item.color
            mapped.confidence = item.confidence
            mapped.position = Point(x=item.x, y=item.y, z=0.0)
            mapped.observation_count = item.observation_count
            mapped.last_seen = Time(
                nanoseconds=int(item.last_seen * 1_000_000_000)
            ).to_msg()
            mapped.state = item.state
            message.objects.append(mapped)

        current = self._field.envelope(
            self._minimum_observations,
            occluded_after=self._occluded_after,
            stale_after=self._stale_after,
        )
        if current is not None:
            message.current_envelope_valid = True
            message.current_envelope_center = Point(x=current.x, y=current.y, z=0.0)
            message.current_envelope_radius = current.radius + self._cylinder_radius
        if self._locked_envelope is not None:
            locked = self._locked_envelope
            message.initial_envelope_locked = True
            message.initial_envelope_center = Point(x=locked.x, y=locked.y, z=0.0)
            message.initial_envelope_radius = locked.radius + self._cylinder_radius
        self._publisher.publish(message)
        self._publish_markers(message)

    def _lock_envelope(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        cylinders = self._field.active(
            self._occluded_after,
            self._stale_after,
        )
        eligible = [
            item
            for item in cylinders
            if item.observation_count >= self._minimum_observations
        ]
        if len(eligible) < self._minimum_lock_objects:
            response.success = False
            response.message = (
                f"need {self._minimum_lock_objects} stable objects; "
                f"currently {len(eligible)}"
            )
            return response
        self._locked_envelope = self._field.envelope(
            self._minimum_observations,
            occluded_after=self._occluded_after,
            stale_after=self._stale_after,
        )
        response.success = self._locked_envelope is not None
        response.message = "initial envelope locked"
        return response

    def _reset(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        del request
        self._field.clear()
        self._locked_envelope = None
        response.success = True
        response.message = "cylinder field reset"
        return response

    def _publish_markers(self, field: CylinderField) -> None:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for item in field.objects:
            marker = Marker()
            marker.header = field.header
            marker.ns = "cylinders"
            marker.id = int(item.cylinder_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = item.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self._cylinder_radius
            marker.scale.y = 2.0 * self._cylinder_radius
            marker.scale.z = 0.25
            marker.color.a = 0.9
            marker.color.r, marker.color.g, marker.color.b = _color_rgb(item.color)
            if item.state == "occluded":
                marker.color.a = 0.35
            markers.markers.append(marker)
        if field.current_envelope_valid:
            markers.markers.append(
                _circle_marker(field, 10001, "current_envelope", False)
            )
        if field.initial_envelope_locked:
            markers.markers.append(
                _circle_marker(field, 10002, "initial_envelope", True)
            )
        self._marker_publisher.publish(markers)


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


def _color_rgb(color: str) -> tuple[float, float, float]:
    return {
        "blue": (0.1, 0.3, 1.0),
        "green": (0.1, 1.0, 0.2),
        "red": (1.0, 0.1, 0.1),
    }.get(color, (1.0, 1.0, 1.0))


def _circle_marker(
    field: CylinderField,
    marker_id: int,
    namespace: str,
    locked: bool,
) -> Marker:
    marker = Marker()
    marker.header = field.header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    if locked:
        marker.pose.position = field.initial_envelope_center
        radius = field.initial_envelope_radius
        marker.color.r = 1.0
        marker.color.g = 0.5
    else:
        marker.pose.position = field.current_envelope_center
        radius = field.current_envelope_radius
        marker.color.g = 0.8
        marker.color.b = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.z = -0.005
    marker.scale.x = 2.0 * radius
    marker.scale.y = 2.0 * radius
    marker.scale.z = 0.01
    marker.color.a = 0.25
    return marker


def main(args=None) -> None:
    rclpy.init(args=args)
    node: CylinderFieldMapperNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = CylinderFieldMapperNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
