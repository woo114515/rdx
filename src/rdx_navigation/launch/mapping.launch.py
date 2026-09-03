"""Launch MS200P, safety arbitration, and SLAM Toolbox mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("rdx_bringup")
    navigation_share = get_package_share_directory("rdx_navigation")
    slam_share = get_package_share_directory("slam_toolbox")

    start_hardware = LaunchConfiguration("start_hardware")
    footprint_verified = LaunchConfiguration("footprint_verified")
    emergency_stop_on_start = LaunchConfiguration("emergency_stop_on_start")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    slam_params_file = LaunchConfiguration("slam_params_file")
    safety_params_file = LaunchConfiguration("safety_params_file")

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "hardware.launch.py")
        ),
        condition=IfCondition(start_hardware),
        launch_arguments={
            "footprint_verified": footprint_verified,
            "emergency_stop_on_start": emergency_stop_on_start,
            "max_linear_speed": max_linear_speed,
            "safety_params_file": safety_params_file,
        }.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "slam_params_file": slam_params_file,
            "use_sim_time": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_hardware", default_value="true"),
            DeclareLaunchArgument("footprint_verified", default_value="false"),
            DeclareLaunchArgument(
                "emergency_stop_on_start", default_value="true"
            ),
            DeclareLaunchArgument("max_linear_speed", default_value="0.12"),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    navigation_share, "config", "slam_toolbox.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=os.path.join(
                    get_package_share_directory("rdx_safety"),
                    "config",
                    "safety.yaml",
                ),
            ),
            hardware,
            slam,
            SetRemap(src="/cmd_vel_teleop", dst="/cmd_vel_teleop"),
        ]
    )
