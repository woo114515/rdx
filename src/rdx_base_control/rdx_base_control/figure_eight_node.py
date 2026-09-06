"""Explicitly armed, TF-closed-loop smooth figure-eight node."""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from .figure_eight_logic import Config, Pose, Tracker

class FigureEightNode(Node):
    PERIOD = 0.05
    TF_FAILURE_LIMIT = 0.25

    def __init__(self):
        super().__init__("rdx_figure_eight")
        self.declare_parameter("task_timeout", 90.0)
        self.declare_parameter("cross_track_limit", 0.50)
        self.task_timeout = float(self.get_parameter("task_timeout").value)
        cross_track_limit = float(
            self.get_parameter("cross_track_limit").value
        )
        if not math.isfinite(self.task_timeout) or self.task_timeout <= 0.0:
            raise ValueError(
                "task_timeout must be finite and greater than zero"
            )
        if (
            not math.isfinite(cross_track_limit)
            or cross_track_limit <= 0.0
        ):
            raise ValueError(
                "cross_track_limit must be finite and greater than zero"
            )
        self.publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.tracker = Tracker(
            Config(cross_track_limit=cross_track_limit)
        )
        self.last_tick = time.monotonic()
        self.started_at = None
        self.tf_missing_since = None
        self.create_service(Trigger, "~/start", self.start)
        self.create_service(Trigger, "~/cancel", self.cancel)
        self.create_timer(self.PERIOD, self.tick)
        self.get_logger().info("Ready; call /rdx_figure_eight/start to arm")

    @staticmethod
    def yaw(rotation):
        return math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                          1-2*(rotation.y**2+rotation.z**2))

    def pose(self):
        transform = self.buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        translation, rotation = transform.transform.translation, transform.transform.rotation
        return Pose(translation.x, translation.y, self.yaw(rotation))

    def start(self, request, response):
        del request
        if self.publisher.get_subscription_count() == 0:
            response.success, response.message = False, "no /cmd_vel_nav subscriber"
            return response
        try:
            pose = self.pose()
        except TransformException as error:
            response.success, response.message = False, f"TF unavailable: {error}"
            return response
        self.tracker.start(pose)
        self.started_at = self.last_tick = time.monotonic()
        response.success, response.message = True, "figure eight started"
        return response

    def cancel(self, request, response):
        del request
        self.tracker.stop()
        self.publish_zero(5)
        response.success, response.message = True, "stopped"
        return response

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        if not self.tracker.active:
            self.publish_zero()
            return
        if self.started_at is not None and now-self.started_at > self.task_timeout:
            self.fail("task timeout")
            return
        try:
            pose = self.pose()
            self.tf_missing_since = None
        except TransformException as error:
            self.publish_zero()
            self.tf_missing_since = self.tf_missing_since or now
            if now-self.tf_missing_since > self.TF_FAILURE_LIMIT:
                self.fail(f"TF unavailable: {error}")
            return
        command = self.tracker.update(pose, dt)
        message = Twist()
        message.linear.x, message.angular.z = command.linear_x, command.angular_z
        self.publisher.publish(message)
        if self.tracker.completed:
            self.publish_zero(5)
            self.get_logger().info("Figure eight complete")

    def fail(self, reason):
        self.tracker.stop()
        self.publish_zero(5)
        self.get_logger().error(reason)

    def publish_zero(self, count=1):
        for _ in range(count):
            self.publisher.publish(Twist())

    def destroy_node(self):
        self.tracker.stop()
        self.publish_zero(5)
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
