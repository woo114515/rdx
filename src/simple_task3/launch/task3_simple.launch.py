"""Start perception and the isolated simple Task 3 controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    perception_share = Path(get_package_share_directory("color_object_sorter"))
    snapshot_share = Path(get_package_share_directory("cylinder_field_mapping"))
    simple_share = Path(get_package_share_directory("simple_task3"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "perception_config",
                default_value=str(perception_share / "config" / "perception.yaml"),
            ),
            DeclareLaunchArgument(
                "snapshot_config",
                default_value=str(snapshot_share / "config" / "snapshot.yaml"),
            ),
            DeclareLaunchArgument(
                "task_config",
                default_value=str(simple_share / "config" / "task3_simple.yaml"),
            ),
            DeclareLaunchArgument(
                "amcl_config",
                default_value=str(simple_share / "config" / "task3_amcl.yaml"),
            ),
            DeclareLaunchArgument(
                "handoff_config",
                default_value=str(
                    simple_share / "config" / "localization_handoff.yaml"
                ),
            ),
            DeclareLaunchArgument("enable_motion", default_value="false"),
            DeclareLaunchArgument(
                "fastdds_builtin_transports", default_value="UDPv4"
            ),
            SetEnvironmentVariable(
                "FASTDDS_BUILTIN_TRANSPORTS",
                LaunchConfiguration("fastdds_builtin_transports"),
            ),
            Node(
                package="cylinder_field_mapping",
                executable="snapshot_builder",
                name="cylinder_snapshot_builder",
                output="screen",
                parameters=[LaunchConfiguration("snapshot_config")],
            ),
            Node(
                package="color_object_sorter",
                executable="color_object_detector",
                name="color_object_detector",
                output="screen",
                parameters=[LaunchConfiguration("perception_config")],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                parameters=[LaunchConfiguration("amcl_config")],
                remappings=[("map", "/task3/fixed_map"), ("scan", "/scan")],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="task3_localization_lifecycle_manager",
                output="screen",
                parameters=[LaunchConfiguration("amcl_config")],
            ),
            Node(
                package="simple_task3",
                executable="localization_handoff",
                name="task3_localization_handoff",
                output="screen",
                parameters=[LaunchConfiguration("handoff_config")],
            ),
            Node(
                package="color_object_sorter",
                executable="candidate_validator",
                name="cylinder_candidate_validator",
                output="screen",
                parameters=[LaunchConfiguration("perception_config")],
            ),
            Node(
                package="simple_task3",
                executable="simple_task_controller",
                name="simple_task_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("task_config"),
                    {
                        "execution_enabled": ParameterValue(
                            LaunchConfiguration("enable_motion"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
