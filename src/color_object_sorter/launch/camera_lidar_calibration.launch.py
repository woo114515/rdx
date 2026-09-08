from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("color_object_sorter"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=str(package_share / "config" / "perception.yaml"),
            ),
            DeclareLaunchArgument(
                "output_file",
                default_value="/home/sunrise/camera_lidar_calibration.csv",
            ),
            Node(
                package="color_object_sorter",
                executable="camera_lidar_capture",
                name="camera_lidar_calibration_capture",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {"output_file": LaunchConfiguration("output_file")},
                ],
            ),
        ]
    )
