"""Start only the hardware needed for mapping and navigation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    safety_share = get_package_share_directory("rdx_safety")
    description_share = get_package_share_directory("yahboomcar_description")

    start_driver = LaunchConfiguration("start_driver")
    start_lidar = LaunchConfiguration("start_lidar")
    start_description = LaunchConfiguration("start_description")
    footprint_verified = LaunchConfiguration("footprint_verified")
    emergency_stop_on_start = LaunchConfiguration("emergency_stop_on_start")
    max_linear_speed = LaunchConfiguration("max_linear_speed")
    safety_params_file = LaunchConfiguration("safety_params_file")

    base_node = Node(
        package="yahboomcar_base_node",
        executable="base_node",
        name="base_node",
        output="screen",
        parameters=[{"pub_odom_tf": True}],
        condition=IfCondition(start_driver),
    )
    driver_node = Node(
        package="yahboomcar_bringup",
        executable="Mcnamu_driver",
        name="driver_node",
        output="screen",
        condition=IfCondition(start_driver),
    )
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, "launch", "description_launch.py")
        ),
        condition=IfCondition(start_description),
    )
    lidar = Node(
        package="oradar_lidar",
        executable="oradar_scan",
        name="MS200",
        output="screen",
        parameters=[
            {
                "device_model": "MS200",
                "frame_id": "lidar_link",
                "scan_topic": "scan",
                "port_name": "/dev/oradar",
                "baudrate": 230400,
                "angle_min": 0.0,
                "angle_max": 0.0,
                "range_min": 0.15,
                "range_max": 20.0,
                "clockwise": False,
                "motor_speed": 10,
            }
        ],
        remappings=[("scan", "/scan")],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(start_lidar),
    )
    safety = Node(
        package="rdx_safety",
        executable="rdx_safety_node",
        name="rdx_safety",
        output="screen",
        parameters=[
            safety_params_file,
            {
                "footprint_verified": ParameterValue(
                    footprint_verified, value_type=bool
                ),
                "emergency_stop_on_start": ParameterValue(
                    emergency_stop_on_start, value_type=bool
                ),
                "max_linear_speed": ParameterValue(
                    max_linear_speed, value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_driver", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("footprint_verified", default_value="false"),
            DeclareLaunchArgument(
                "emergency_stop_on_start", default_value="true"
            ),
            DeclareLaunchArgument("max_linear_speed", default_value="0.18"),
            DeclareLaunchArgument(
                "safety_params_file",
                default_value=os.path.join(
                    safety_share, "config", "safety.yaml"
                ),
            ),
            description,
            base_node,
            TimerAction(period=0.5, actions=[driver_node]),
            lidar,
            safety,
        ]
    )
