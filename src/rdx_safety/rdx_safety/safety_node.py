"""ROS adapter for the fail-closed velocity safety controller."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from .safety_logic import (
    SafetyConfig,
    SafetyController,
    SafetyDecision,
    Scan,
    Velocity,
    heartbeat_is_stale,
)


class SafetyNode(Node):
    """Route teleoperation or navigation requests through safety gates."""

    def __init__(self) -> None:
        super().__init__("rdx_safety")
        self._declare_parameters()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._controller = SafetyController(self._load_config())
        self._footprint_verified = bool(
            self.get_parameter("footprint_verified").value
        )
        self._emergency_stop = bool(
            self.get_parameter("emergency_stop_on_start").value
        )
        self._emergency_stop_stamp: Optional[float] = None
        self._emergency_stop_timeout = float(
            self.get_parameter("emergency_stop_timeout").value
        )
        if (
            not math.isfinite(self._emergency_stop_timeout)
            or self._emergency_stop_timeout <= 0.0
        ):
            raise ValueError(
                "emergency_stop_timeout must be finite and greater than zero"
            )

        self._nav_command: Optional[Velocity] = None
        self._nav_stamp: Optional[float] = None
        self._teleop_command: Optional[Velocity] = None
        self._teleop_stamp: Optional[float] = None
        self._scan: Optional[Scan] = None
        self._scan_stamp: Optional[float] = None
        self._last_state = ""

        output_topic = str(self.get_parameter("output_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        ready_topic = str(self.get_parameter("ready_topic").value)
        self._velocity_publisher = self.create_publisher(Twist, output_topic, 10)
        self._state_publisher = self.create_publisher(String, state_topic, 10)
        self._ready_publisher = self.create_publisher(Bool, ready_topic, 10)

        self.create_subscription(
            Twist,
            str(self.get_parameter("nav_topic").value),
            self._on_nav_command,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("teleop_topic").value),
            self._on_teleop_command,
            10,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._on_emergency_stop,
            10,
        )

        publish_rate = float(self.get_parameter("publish_rate").value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and greater than zero")
        self.create_timer(1.0 / publish_rate, self._publish_decision)

        if self._emergency_stop:
            self.get_logger().warning(
                "Emergency stop starts engaged; publish false to release it "
                "after completing the safety checklist."
            )
        if not self._footprint_verified:
            self.get_logger().warning(
                "Footprint is unverified; all non-zero velocity is blocked."
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "publish_rate": 20.0,
            "command_timeout": 0.25,
            "scan_timeout": 0.30,
            "max_linear_speed": 0.18,
            "max_angular_speed": 0.60,
            "stop_distance": 0.35,
            "slow_distance": 0.60,
            "sector_half_angle_degrees": 30.0,
            "rotation_stop_distance": 0.35,
            "footprint_verified": False,
            "emergency_stop_on_start": True,
            "emergency_stop_timeout": 0.75,
            "nav_topic": "/cmd_vel_nav",
            "teleop_topic": "/cmd_vel_teleop",
            "scan_topic": "/scan",
            "emergency_stop_topic": "/emergency_stop",
            "output_topic": "/cmd_vel_safe",
            "state_topic": "/rdx_safety/state",
            "ready_topic": "/rdx_safety/ready",
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _load_config(self) -> SafetyConfig:
        return SafetyConfig(
            command_timeout=float(self.get_parameter("command_timeout").value),
            scan_timeout=float(self.get_parameter("scan_timeout").value),
            max_linear_speed=float(
                self.get_parameter("max_linear_speed").value
            ),
            max_angular_speed=float(
                self.get_parameter("max_angular_speed").value
            ),
            stop_distance=float(self.get_parameter("stop_distance").value),
            slow_distance=float(self.get_parameter("slow_distance").value),
            sector_half_angle=math.radians(
                float(self.get_parameter("sector_half_angle_degrees").value)
            ),
            rotation_stop_distance=float(
                self.get_parameter("rotation_stop_distance").value
            ),
        )

    def _now(self) -> float:
        return self._steady_clock.now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _from_twist(message: Twist) -> Velocity:
        return Velocity(
            linear_x=message.linear.x,
            linear_y=message.linear.y,
            angular_z=message.angular.z,
        )

    @staticmethod
    def _to_twist(velocity: Velocity) -> Twist:
        message = Twist()
        message.linear.x = velocity.linear_x
        message.linear.y = velocity.linear_y
        message.angular.z = velocity.angular_z
        return message

    def _on_nav_command(self, message: Twist) -> None:
        self._nav_command = self._from_twist(message)
        self._nav_stamp = self._now()

    def _on_teleop_command(self, message: Twist) -> None:
        self._teleop_command = self._from_twist(message)
        self._teleop_stamp = self._now()

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = Scan(
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            ranges=message.ranges,
            range_min=message.range_min,
            range_max=message.range_max,
        )
        self._scan_stamp = self._now()

    def _on_emergency_stop(self, message: Bool) -> None:
        self._emergency_stop = bool(message.data)
        self._emergency_stop_stamp = self._now()
        level = self.get_logger().warning if message.data else self.get_logger().info
        level("Emergency stop engaged." if message.data else "Emergency stop released.")

    def _publish_decision(self) -> None:
        now = self._now()
        release_timed_out = (
            not self._emergency_stop
            and heartbeat_is_stale(
                now,
                self._emergency_stop_stamp,
                self._emergency_stop_timeout,
            )
        )
        if release_timed_out:
            decision = SafetyDecision(
                Velocity.zero(), "emergency_stop_timeout", "none"
            )
        else:
            decision = self._controller.evaluate(
                now=now,
                nav_command=self._nav_command,
                nav_stamp=self._nav_stamp,
                teleop_command=self._teleop_command,
                teleop_stamp=self._teleop_stamp,
                scan=self._scan,
                scan_stamp=self._scan_stamp,
                emergency_stop=self._emergency_stop,
                footprint_verified=self._footprint_verified,
            )
        self._velocity_publisher.publish(self._to_twist(decision.velocity))

        ready = decision.reason in {"idle", "clear", "obstacle_slow"}
        self._ready_publisher.publish(Bool(data=ready))
        clearance = (
            "unknown"
            if decision.clearance is None
            else f"{decision.clearance:.3f}"
        )
        state = (
            f"reason={decision.reason};source={decision.source};"
            f"clearance={clearance}"
        )
        self._state_publisher.publish(String(data=state))
        if state != self._last_state:
            self.get_logger().info(state)
            self._last_state = state

    def publish_stop(self, repetitions: int = 3) -> None:
        stop = self._to_twist(Velocity.zero())
        for _ in range(repetitions):
            try:
                self._velocity_publisher.publish(stop)
            except Exception as error:  # ROS context may already be shutting down.
                self.get_logger().error(f"Failed to publish shutdown stop: {error}")
                break


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[SafetyNode] = None
    try:
        node = SafetyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
