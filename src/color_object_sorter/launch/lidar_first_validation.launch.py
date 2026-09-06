from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("color_object_sorter"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config" / "perception.yaml")
            ),
            Node(
                package="color_object_sorter",
                executable="color_object_detector",
                name="color_object_detector",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
            Node(
                package="color_object_sorter",
                executable="candidate_validator",
                name="cylinder_candidate_validator",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
