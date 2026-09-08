"""Start the proven Task 3 perception chain and one compact task controller."""

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
    planner_share = Path(get_package_share_directory("cylinder_push_planner"))

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
                "direct_task_config",
                default_value=str(planner_share / "config" / "direct_task.yaml"),
            ),
            DeclareLaunchArgument(
                "enable_motion",
                default_value="false",
                description="Allow run_once to publish low-speed motion commands",
            ),
            DeclareLaunchArgument(
                "fastdds_builtin_transports",
                default_value="UDPv4",
                description="Avoid stale Fast DDS shared-memory port locks",
            ),
            SetEnvironmentVariable(
                "FASTDDS_BUILTIN_TRANSPORTS",
                LaunchConfiguration("fastdds_builtin_transports"),
            ),
            # Keep the already validated six-cylinder perception chain intact.
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
                package="color_object_sorter",
                executable="candidate_validator",
                name="cylinder_candidate_validator",
                output="screen",
                parameters=[LaunchConfiguration("perception_config")],
            ),
            Node(
                package="cylinder_push_planner",
                executable="direct_task_controller",
                name="cylinder_direct_task_controller",
                output="screen",
                parameters=[
                    LaunchConfiguration("direct_task_config"),
                    {
                        "execution_enabled": ParameterValue(
                            LaunchConfiguration("enable_motion"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
