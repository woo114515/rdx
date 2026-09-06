from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="cylinder_push_planner",
                executable="selection_planner",
                output="screen",
            )
        ]
    )
