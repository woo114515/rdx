"""Launch static-map Nav2 navigation and the safety-routed hardware."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml


def _validate_map(context):
    map_path = Path(LaunchConfiguration("map").perform(context))
    if not map_path.is_file() or map_path.suffix.lower() not in {".yaml", ".yml"}:
        raise RuntimeError(
            f"map must be an existing .yaml/.yml file, got {map_path}"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("rdx_bringup")
    navigation_share = get_package_share_directory("rdx_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")

    start_hardware = LaunchConfiguration("start_hardware")
    footprint_verified = LaunchConfiguration("footprint_verified")
    emergency_stop_on_start = LaunchConfiguration("emergency_stop_on_start")
    safety_params_file = LaunchConfiguration("safety_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    map_file = LaunchConfiguration("map")
    behavior_tree_file = os.path.join(
        navigation_share, "behavior_trees", "no_recovery_nav_to_pose.xml"
    )
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": behavior_tree_file,
        },
        convert_types=True,
    )

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "hardware.launch.py")
        ),
        condition=IfCondition(start_hardware),
        launch_arguments={
            "footprint_verified": footprint_verified,
            "emergency_stop_on_start": emergency_stop_on_start,
            "safety_params_file": safety_params_file,
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": "false",
            "params_file": configured_params,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map"),
            DeclareLaunchArgument("start_hardware", default_value="true"),
            DeclareLaunchArgument("footprint_verified", default_value="false"),
            DeclareLaunchArgument(
                "emergency_stop_on_start", default_value="true"
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    navigation_share, "config", "nav2.yaml"
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
            OpaqueFunction(function=_validate_map),
            hardware,
            GroupAction(
                actions=[
                    SetRemap(src="/cmd_vel", dst="/cmd_vel_nav"),
                    nav2,
                ]
            ),
        ]
    )
