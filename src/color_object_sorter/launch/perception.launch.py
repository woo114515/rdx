from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("color_object_sorter"))
    default_config = str(package_share / "config" / "perception.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="color_object_sorter",
                executable="color_object_detector",
                name="color_object_detector",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
