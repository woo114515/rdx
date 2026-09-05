"""ROS 2 safety boundary between sorting requests and the mobile base."""

from __future__ import annotations

import json
import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .behavior import VelocityCommand
from .safety import (
    CommandWatchdog,
    ObstacleConfig,
    SafetyConfig,
    command_has_obstacle,
    navigation_controls_output,
    output_topic_has_conflict,
)


class SafetyArbiterNode(Node):
    """Limit sorting commands and stop on stale input or emergency stop."""

    def __init__(self) -> None:
        super().__init__("rdx_sorting_safety")
        self.declare_parameter("output_enabled", False)
        self.declare_parameter(
            "request_topic",
            "/rdx_sorting/cmd_vel_request",
        )
        self.declare_parameter("navigation_request_enabled", False)
        self.declare_parameter(
            "navigation_request_topic",
            "/rdx_sorting/nav_cmd_vel_request",
        )
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("stop_on_output_conflict", True)
        self.declare_parameter("lidar_stop_enabled", False)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("obstacle_stop_distance", 0.30)
        self.declare_parameter("movement_half_angle_degrees", 35.0)
        self.declare_parameter("target_corridor_half_angle_degrees", 15.0)
        self.declare_parameter("sorting_status_topic", "/rdx_sorting/status")
        self.declare_parameter(
            "emergency_stop_topic",
            "/rdx_sorting/emergency_stop",
        )
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("request_timeout", 0.3)
        self.declare_parameter("maximum_linear_speed", 0.12)
        self.declare_parameter("maximum_angular_speed", 0.25)

        self._output_enabled = bool(
            self.get_parameter("output_enabled").value
        )
        self._stop_on_output_conflict = bool(
            self.get_parameter("stop_on_output_conflict").value
        )
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._navigation_request_enabled = bool(
            self.get_parameter("navigation_request_enabled").value
        )
        self._lidar_stop_enabled = bool(
            self.get_parameter("lidar_stop_enabled").value
        )
        self._scan_timeout = float(self.get_parameter("scan_timeout").value)
        if self._scan_timeout <= 0.0:
            raise ValueError("scan_timeout must be positive")
        self._obstacle_config = ObstacleConfig(
            stop_distance=float(
                self.get_parameter("obstacle_stop_distance").value
            ),
            movement_half_angle=math.radians(
                float(
                    self.get_parameter("movement_half_angle_degrees").value
                )
            ),
            target_corridor_half_angle=math.radians(
                float(
                    self.get_parameter(
                        "target_corridor_half_angle_degrees"
                    ).value
                )
            ),
        )
        self._last_scan: LaserScan | None = None
        self._last_scan_at: float | None = None
        self._sorting_phase = ""
        self._lidar_stop_reason = ""
        safety_config = SafetyConfig(
            request_timeout=float(self.get_parameter("request_timeout").value),
            maximum_linear_speed=float(
                self.get_parameter("maximum_linear_speed").value
            ),
            maximum_angular_speed=float(
                self.get_parameter("maximum_angular_speed").value
            ),
        )
        self._sorting_watchdog = CommandWatchdog(safety_config)
        self._navigation_watchdog = CommandWatchdog(safety_config)
        self._publisher = self.create_publisher(
            Twist,
            self._output_topic,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("request_topic").value),
            self._on_request,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("navigation_request_topic").value),
            self._on_navigation_request,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._on_emergency_stop,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("sorting_status_topic").value),
            self._on_sorting_status,
            10,
        )
        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")
        self.create_timer(1.0 / publish_rate, self._on_timer)
        self.get_logger().info(
            f"Sorting safety ready; output_enabled={self._output_enabled}"
        )

    def _on_request(self, message: Twist) -> None:
        self._sorting_watchdog.update(
            self._now(),
            VelocityCommand(
                linear_x=message.linear.x,
                linear_y=message.linear.y,
                angular_z=message.angular.z,
            ),
        )

    def _on_navigation_request(self, message: Twist) -> None:
        self._navigation_watchdog.update(
            self._now(),
            VelocityCommand(
                linear_x=message.linear.x,
                linear_y=message.linear.y,
                angular_z=message.angular.z,
            ),
        )

    def _on_emergency_stop(self, message: Bool) -> None:
        if message.data and not self._sorting_watchdog.emergency_stopped:
            self._sorting_watchdog.emergency_stop()
            self._navigation_watchdog.emergency_stop()
            self.get_logger().error(
                "Emergency stop latched; restart is required to re-arm"
            )
            if self._output_enabled:
                self._publish(VelocityCommand())

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan = message
        self._last_scan_at = self._now()

    def _on_sorting_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._sorting_phase = str(status.get("state", ""))

    def _on_timer(self) -> None:
        if not self._output_enabled:
            return
        if (
            self._stop_on_output_conflict
            and output_topic_has_conflict(
                self.count_publishers(self._output_topic)
            )
            and not self._sorting_watchdog.emergency_stopped
        ):
            self._sorting_watchdog.emergency_stop()
            self._navigation_watchdog.emergency_stop()
            self.get_logger().error(
                "Another publisher is using the base command topic; "
                "motion output is latched off until restart"
            )
        now = self._now()
        if navigation_controls_output(
            self._navigation_request_enabled,
            self._sorting_phase,
        ):
            command = self._navigation_watchdog.output(now)
        else:
            command = self._sorting_watchdog.output(now)
        if self._lidar_stop_enabled:
            reason = self._lidar_block_reason(now, command)
            self._report_lidar_stop(reason)
            if reason:
                command = VelocityCommand()
        self._publish(command)

    def _lidar_block_reason(
        self,
        now: float,
        command: VelocityCommand,
    ) -> str:
        if self._last_scan is None or self._last_scan_at is None:
            return "waiting for laser scan"
        if now - self._last_scan_at > self._scan_timeout:
            return "laser scan timeout"
        scan = self._last_scan
        if command_has_obstacle(
            command,
            scan.ranges,
            angle_min=scan.angle_min,
            angle_increment=scan.angle_increment,
            range_min=scan.range_min,
            range_max=scan.range_max,
            config=self._obstacle_config,
            ignore_forward_target_corridor=(
                self._sorting_phase in {"approach", "push"}
            ),
        ):
            return "obstacle in movement direction"
        return ""

    def _report_lidar_stop(self, reason: str) -> None:
        if reason == self._lidar_stop_reason:
            return
        if reason:
            self.get_logger().warning(f"Motion stopped: {reason}")
        elif self._lidar_stop_reason:
            self.get_logger().info("Laser safety path is clear")
        self._lidar_stop_reason = reason

    def _publish(self, command: VelocityCommand) -> None:
        message = Twist()
        message.linear.x = command.linear_x
        message.linear.y = command.linear_y
        message.angular.z = command.angular_z
        self._publisher.publish(message)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def destroy_node(self) -> bool:
        if self._output_enabled:
            self._publish(VelocityCommand())
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetyArbiterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
