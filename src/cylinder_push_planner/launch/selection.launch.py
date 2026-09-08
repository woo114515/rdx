from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("cylinder_push_planner"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "selection_config",
                default_value=str(share / "config" / "selection.yaml"),
            ),
            Node(
                package="cylinder_push_planner",
                executable="selection_planner",
                output="screen",
                parameters=[LaunchConfiguration("selection_config")],
            )
        ]
    )
