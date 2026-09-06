from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("cylinder_field_mapping"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config", default_value=str(share / "config" / "snapshot.yaml")
            ),
            Node(
                package="cylinder_field_mapping",
                executable="snapshot_builder",
                name="cylinder_snapshot_builder",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
