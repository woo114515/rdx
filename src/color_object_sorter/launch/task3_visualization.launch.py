"""Open the lightweight Task 3 cylinder visualization in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("color_object_sorter"))
    rviz_config = package_share / "rviz" / "task3_cylinders.rviz"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fastdds_builtin_transports",
                default_value="UDPv4",
                description="Avoid Fast DDS shared-memory port-lock conflicts",
            ),
            SetEnvironmentVariable(
                "FASTDDS_BUILTIN_TRANSPORTS",
                LaunchConfiguration("fastdds_builtin_transports"),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="task3_cylinder_visualization",
                output="screen",
                arguments=["-d", str(rviz_config)],
            )
        ]
    )
