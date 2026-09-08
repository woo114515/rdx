"""ROS node building an explicit, LiDAR-first cylinder candidate snapshot."""

from __future__ import annotations

import json
import math

import rclpy
from color_object_sorter_interfaces.msg import (
    CylinderCandidate,
    CylinderSnapshot,
)
from color_object_sorter_interfaces.srv import (
    SetObjectInventory,
    SetSnapshotExclusions,
)
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .lidar_candidates import (
    CandidateAccumulator,
    extract_candidates,
    merge_nearby_observations,
    select_primary_spatial_group,
)
from .inventory import ObjectInventory
from .map_filter import GridMap, inside_exclusion_zone, is_compact_map_obstacle


class CylinderSnapshotNode(Node):
    """Collect a bounded candidate snapshot without producing motion."""

    def __init__(self) -> None:
        super().__init__("cylinder_snapshot_builder")
        defaults = {
            "scan_topic": "/scan",
            "output_topic": "/cylinder_snapshot/candidates",
            "marker_topic": "/cylinder_snapshot/markers",
            "filter_diagnostics_topic": "/cylinder_snapshot/filter_diagnostics",
            "map_topic": "/map",
            "fixed_frame": "map",
            "minimum_object_count": 1,
            "minimum_observations": 4,
            "association_distance": 0.15,
            "minimum_object_separation": 0.20,
            "history_size": 30,
            "maximum_missed_updates": 5,
            "maximum_point_gap": 0.08,
            "minimum_cluster_points": 2,
            "minimum_diameter": 0.015,
            "maximum_diameter": 0.16,
            "nominal_radius": 0.035,
            "transform_timeout": 0.15,
            "use_map_filter": False,
            "require_map_filter": True,
            "map_occupancy_threshold": 50,
            "map_search_radius": 0.12,
            "maximum_map_component_diameter": 0.20,
            "observation_min_range": 0.25,
            "observation_max_range": 4.0,
            "observation_half_angle": 0.5235987756,
            "observation_max_lateral": 2.0,
            "maximum_group_neighbor_distance": 0.65,
            "use_workspace_bounds": False,
            "workspace_min_x": -10.0,
            "workspace_max_x": 10.0,
            "workspace_min_y": -10.0,
            "workspace_max_y": 10.0,
            "minimum_exclusion_radius": 0.10,
            "maximum_exclusion_radius": 1.00,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter("inventory_colors", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("inventory_counts", Parameter.Type.INTEGER_ARRAY)

        self._inventory = ObjectInventory.from_lists(
            self.get_parameter_or(
                "inventory_colors",
                Parameter(
                    "inventory_colors", Parameter.Type.STRING_ARRAY, []
                ),
            ).value,
            self.get_parameter_or(
                "inventory_counts",
                Parameter(
                    "inventory_counts", Parameter.Type.INTEGER_ARRAY, []
                ),
            ).value,
        )

        self._accumulator = CandidateAccumulator(
            self._float("association_distance"),
            self._int("history_size"),
            self._int("maximum_missed_updates"),
        )
        self._collecting = False
        self._locked = False
        self._map: GridMap | None = None
        self._exclusions: tuple[tuple[float, float], ...] = ()
        self._exclusion_radius = 0.0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            CylinderSnapshot, self._string("output_topic"), 10
        )
        self._marker_publisher = self.create_publisher(
            MarkerArray, self._string("marker_topic"), 10
        )
        diagnostics_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._diagnostics_publisher = self.create_publisher(
            String,
            self._string("filter_diagnostics_topic"),
            diagnostics_qos,
        )
        self.create_subscription(
            LaserScan,
            self._string("scan_topic"),
            self._on_scan,
            qos_profile_sensor_data,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self._string("map_topic"), self._on_map, map_qos
        )
        self.create_service(Trigger, "/cylinder_snapshot/start", self._start)
        self.create_service(Trigger, "/cylinder_snapshot/lock", self._lock)
        self.create_service(Trigger, "/cylinder_snapshot/reset", self._reset)
        self.create_service(
            SetObjectInventory,
            "/cylinder_snapshot/set_inventory",
            self._set_inventory,
        )
        self.create_service(
            SetSnapshotExclusions,
            "/cylinder_snapshot/set_exclusions",
            self._set_exclusions,
        )
        self.get_logger().info(
            "LiDAR-first snapshot builder ready; motion output is absent"
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        orientation = message.info.origin.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        self._map = GridMap(
            width=message.info.width,
            height=message.info.height,
            resolution=message.info.resolution,
            origin_x=message.info.origin.position.x,
            origin_y=message.info.origin.position.y,
            origin_yaw=yaw,
            data=message.data,
        )

    def _on_scan(self, scan: LaserScan) -> None:
        if not self._collecting or self._locked:
            return
        fixed_frame = self._string("fixed_frame")
        try:
            transform = self._tf_buffer.lookup_transform(
                fixed_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=self._float("transform_timeout")),
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot transform candidate scan: {error}",
                throttle_duration_sec=2.0,
            )
            self._publish_filter_diagnostics(
                scan,
                {
                    "status": "tf_error",
                    "error": str(error),
                    "map_available": self._map is not None,
                },
            )
            return
        extracted = extract_candidates(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            maximum_point_gap=self._float("maximum_point_gap"),
            minimum_points=self._int("minimum_cluster_points"),
            minimum_diameter=self._float("minimum_diameter"),
            maximum_diameter=self._float("maximum_diameter"),
            nominal_radius=self._float("nominal_radius"),
        )
        counts = {
            "status": "ok",
            "map_available": self._map is not None,
            "raw_clusters": len(extracted),
            "sector_rejected": 0,
            "workspace_rejected": 0,
            "exclusion_rejected": 0,
            "map_rejected": 0,
        }
        observations = []
        for item in extracted:
            if not self._inside_observation_sector(item.x, item.y):
                counts["sector_rejected"] += 1
                continue
            x, y = _transform_xy(item.x, item.y, transform.transform)
            if not self._inside_workspace(x, y):
                counts["workspace_rejected"] += 1
                continue
            if self._inside_exclusion(x, y):
                counts["exclusion_rejected"] += 1
                continue
            if not self._passes_map_filter(x, y):
                counts["map_rejected"] += 1
                continue
            observations.append((x, y, item.radius, item.confidence))
        counts["accepted_before_merge"] = len(observations)
        observations = list(
            merge_nearby_observations(
                observations, self._float("minimum_object_separation")
            )
        )
        counts["merged_observations"] = len(observations)
        counts["merged_away"] = counts["accepted_before_merge"] - len(observations)
        stamp = Time.from_msg(scan.header.stamp).nanoseconds / 1e9
        tracked = self._accumulator.update(observations, stamp)
        stable = self._accumulator.stable(self._int("minimum_observations"))
        selected = select_primary_spatial_group(
            stable, self._float("maximum_group_neighbor_distance")
        )
        counts["tracked_candidates"] = len(tracked)
        counts["unstable_tracks"] = len(tracked) - len(stable)
        counts["stable_candidates"] = len(stable)
        counts["group_rejected"] = len(stable) - len(selected)
        counts["published_candidates"] = len(selected)
        counts["expected_candidates"] = self._inventory.total
        self._publish_filter_diagnostics(scan, counts)
        self._publish(scan.header.stamp, selected)

    def _publish_filter_diagnostics(self, scan: LaserScan, values: dict) -> None:
        payload = {
            "stamp": {
                "sec": int(scan.header.stamp.sec),
                "nanosec": int(scan.header.stamp.nanosec),
            },
            "scan_frame": scan.header.frame_id,
            **values,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._diagnostics_publisher.publish(message)

    def _publish(self, stamp, candidates) -> None:
        message = CylinderSnapshot()
        message.header.stamp = stamp
        message.header.frame_id = self._string("fixed_frame")
        message.expected_object_count = self._inventory.total
        message.expected_colors = list(self._inventory.colors)
        message.expected_color_counts = list(self._inventory.color_counts)
        message.locked = self._locked
        message.ready, message.status = self._readiness(len(candidates))
        for candidate in candidates:
            item = CylinderCandidate()
            item.header = message.header
            item.candidate_id = candidate.candidate_id
            item.position = Point(x=candidate.x, y=candidate.y, z=0.0)
            item.radius = candidate.radius
            item.confidence = candidate.confidence
            item.observation_count = candidate.observation_count
            item.last_seen = Time(
                nanoseconds=int(candidate.last_seen * 1_000_000_000)
            ).to_msg()
            message.candidates.append(item)
        self._publisher.publish(message)
        self._publish_markers(message)

    def _readiness(self, count: int) -> tuple[bool, str]:
        expected = self._inventory.total
        minimum = self._int("minimum_object_count")
        if expected > 0:
            if count < expected:
                return False, f"incomplete:{count}/{expected}"
            if count > expected:
                return False, f"ambiguous:{count}/{expected}"
            return True, f"ready:{count}"
        if count < minimum:
            return False, f"incomplete:{count}/{minimum}"
        return True, f"ready:{count}"

    def _inside_workspace(self, x: float, y: float) -> bool:
        if not bool(self.get_parameter("use_workspace_bounds").value):
            return True
        return (
            self._float("workspace_min_x") <= x <= self._float("workspace_max_x")
            and self._float("workspace_min_y") <= y <= self._float("workspace_max_y")
        )

    def _inside_observation_sector(self, x: float, y: float) -> bool:
        distance = math.hypot(x, y)
        bearing = abs(math.atan2(y, x))
        return (
            self._float("observation_min_range")
            <= distance
            <= self._float("observation_max_range")
            and bearing <= self._float("observation_half_angle")
            and abs(y) <= self._float("observation_max_lateral")
        )

    def _inside_exclusion(self, x: float, y: float) -> bool:
        return inside_exclusion_zone(
            x,
            y,
            self._exclusions,
            self._exclusion_radius,
        )

    def _passes_map_filter(self, x: float, y: float) -> bool:
        if not bool(self.get_parameter("use_map_filter").value):
            return True
        if self._map is None:
            return not bool(self.get_parameter("require_map_filter").value)
        return is_compact_map_obstacle(
            self._map,
            x,
            y,
            occupancy_threshold=self._int("map_occupancy_threshold"),
            search_radius=self._float("map_search_radius"),
            maximum_component_diameter=self._float(
                "maximum_map_component_diameter"
            ),
        )

    def _start(self, request, response):
        del request
        self._accumulator.clear()
        self._locked = False
        self._collecting = True
        response.success = True
        response.message = "new snapshot collection started"
        return response

    def _lock(self, request, response):
        del request
        candidates = self._accumulator.stable(self._int("minimum_observations"))
        expected = self._inventory.total
        enough_for_validation = len(candidates) >= max(
            self._int("minimum_object_count"), expected
        )
        if not enough_for_validation:
            response.success = False
            response.message = f"insufficient candidates:{len(candidates)}"
            return response
        self._locked = True
        self._collecting = False
        response.success = True
        response.message = (
            f"snapshot locked with {len(candidates)} candidates for visual validation"
        )
        return response

    def _reset(self, request, response):
        del request
        self._accumulator.clear()
        self._locked = False
        self._collecting = False
        self._publish(Time().to_msg(), ())
        response.success = True
        response.message = "snapshot reset"
        return response

    def _set_inventory(self, request, response):
        """Replace the expected inventory only while collection is stopped."""

        if self._collecting or self._locked:
            response.success = False
            response.message = "reset the snapshot before changing inventory"
            return response
        try:
            inventory = ObjectInventory.from_lists(request.colors, request.counts)
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        self._inventory = inventory
        self._publish(Time().to_msg(), ())
        response.success = True
        summary = ",".join(
            f"{color}:{count}" for color, count in inventory.counts
        )
        response.message = f"inventory updated:{summary or 'unconstrained'}"
        return response

    def _set_exclusions(self, request, response):
        """Replace delivered-object exclusion zones while collection is stopped."""

        if self._collecting or self._locked:
            response.success = False
            response.message = "reset the snapshot before changing exclusions"
            return response
        radius = float(request.radius)
        if request.centers and not (
            self._float("minimum_exclusion_radius")
            <= radius
            <= self._float("maximum_exclusion_radius")
        ):
            response.success = False
            response.message = "exclusion radius is outside configured limits"
            return response
        if any(
            not math.isfinite(value)
            for center in request.centers
            for value in (center.x, center.y)
        ):
            response.success = False
            response.message = "exclusion centers must be finite"
            return response
        self._exclusions = tuple((center.x, center.y) for center in request.centers)
        self._exclusion_radius = radius if self._exclusions else 0.0
        self._publish(Time().to_msg(), ())
        response.success = True
        response.message = f"configured {len(self._exclusions)} exclusion zones"
        return response

    def _publish_markers(self, snapshot: CylinderSnapshot) -> None:
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        for candidate in snapshot.candidates:
            marker = Marker()
            marker.header = snapshot.header
            marker.ns = "lidar_candidates"
            marker.id = int(candidate.candidate_id)
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = candidate.position
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * candidate.radius
            marker.scale.y = 2.0 * candidate.radius
            marker.scale.z = 0.25
            marker.color.r = 1.0
            marker.color.g = 0.75 if snapshot.ready else 0.25
            marker.color.b = 0.1
            marker.color.a = 0.25
            output.markers.append(marker)
        for index, (center_x, center_y) in enumerate(self._exclusions):
            marker = Marker()
            marker.header = snapshot.header
            marker.ns = "delivered_exclusions"
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position = Point(x=center_x, y=center_y, z=0.005)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 2.0 * self._exclusion_radius
            marker.scale.y = 2.0 * self._exclusion_radius
            marker.scale.z = 0.01
            marker.color.r = 0.55
            marker.color.g = 0.55
            marker.color.b = 0.55
            marker.color.a = 0.35
            output.markers.append(marker)
        self._marker_publisher.publish(output)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CylinderSnapshotNode()
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
