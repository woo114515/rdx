from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("cylinder_field_mapping"))
    return LaunchDescription(
        [
            Node(
                package="cylinder_field_mapping",
                executable="field_mapper",
                name="cylinder_field_mapper",
                output="screen",
                parameters=[str(share / "config" / "field_mapping.yaml")],
            )
        ]
    )
