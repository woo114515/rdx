"""ROS NavigateToPose client for the fixed task-two route."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from .mission_logic import (
    MissionConfigError,
    MissionStateMachine,
    Waypoint,
    load_mission,
    quaternion_from_yaw,
)


class MissionNode(Node):
    """Start and cancel one fixed-order navigation mission."""

    def __init__(self) -> None:
        super().__init__("rdx_mission")
        self.declare_parameter(
            "waypoint_file",
            str(
                Path(get_package_share_directory("rdx_navigation"))
                / "config"
                / "waypoints.yaml"
            ),
        )
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("start_service", "/mission/start")
        self.declare_parameter("cancel_service", "/mission/cancel")

        self._plan_error: Optional[str] = None
        try:
            plan = load_mission(
                str(self.get_parameter("waypoint_file").value)
            )
            self._machine: Optional[MissionStateMachine] = MissionStateMachine(plan)
        except MissionConfigError as error:
            self._machine = None
            self._plan_error = str(error)
            self.get_logger().error(self._plan_error)

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("action_name").value),
        )
        self._pending_goal = None
        self.create_service(
            Trigger,
            str(self.get_parameter("start_service").value),
            self._start_callback,
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("cancel_service").value),
            self._cancel_callback,
        )

    def _start_callback(self, request, response):
        del request
        if self._machine is None:
            response.success = False
            response.message = self._plan_error or "mission configuration unavailable"
            return response
        if self._machine.state != "idle":
            response.success = False
            response.message = f"mission is already {self._machine.state}"
            return response
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            response.success = False
            response.message = "NavigateToPose action server is not available"
            return response

        waypoint = self._machine.start()
        try:
            self._send_goal(waypoint)
        except Exception as error:
            self._machine.goal_failed(str(error))
            response.success = False
            response.message = f"failed to send {waypoint.name}: {error}"
            return response

        response.success = True
        response.message = f"started at {waypoint.name}"
        return response

    def _cancel_callback(self, request, response):
        del request
        if self._machine is None or self._machine.state != "running":
            response.success = False
            response.message = "no running mission"
            return response

        if self._pending_goal is not None:
            self._pending_goal.cancel_goal_async()
        self._machine.cancel()
        response.success = True
        response.message = "mission cancelled; safety node will hold zero if needed"
        return response

    def _send_goal(self, waypoint: Waypoint) -> None:
        assert self._machine is not None
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._machine.plan.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint.x
        goal.pose.pose.position.y = waypoint.y
        (
            goal.pose.pose.orientation.x,
            goal.pose.pose.orientation.y,
            goal.pose.pose.orientation.z,
            goal.pose.pose.orientation.w,
        ) = quaternion_from_yaw(waypoint.yaw)

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info(
            f"sent {waypoint.name}: x={waypoint.x:.3f}, "
            f"y={waypoint.y:.3f}, yaw={waypoint.yaw:.3f}"
        )

    def _goal_response_callback(self, future) -> None:
        if self._machine is None or self._machine.state != "running":
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._machine.goal_failed(str(error))
            self.get_logger().error(f"NavigateToPose request failed: {error}")
            return
        if not goal_handle.accepted:
            self._machine.goal_failed("NavigateToPose goal rejected")
            self.get_logger().error("NavigateToPose goal was rejected")
            return

        self._pending_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        self._pending_goal = None
        if self._machine is None or self._machine.state != "running":
            return
        try:
            status = future.result().status
        except Exception as error:
            self._machine.goal_failed(str(error))
            self.get_logger().error(f"NavigateToPose result failed: {error}")
            return

        if status != GoalStatus.STATUS_SUCCEEDED:
            reason = f"NavigateToPose status {status}"
            self._machine.goal_failed(reason)
            self.get_logger().error(reason)
            return

        next_waypoint = self._machine.goal_succeeded()
        if next_waypoint is None:
            self.get_logger().info("mission completed: returned to start")
            return
        try:
            self._send_goal(next_waypoint)
        except Exception as error:
            self._machine.goal_failed(str(error))
            self.get_logger().error(f"failed to send {next_waypoint.name}: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MissionNode] = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
