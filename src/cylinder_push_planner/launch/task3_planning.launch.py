"""Start Task 3 perception, planning, and default-disabled execution."""

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
                "reobservation_config",
                default_value=str(planner_share / "config" / "reobservation.yaml"),
            ),
            DeclareLaunchArgument(
                "selection_config",
                default_value=str(planner_share / "config" / "selection.yaml"),
            ),
            DeclareLaunchArgument(
                "approach_execution_config",
                default_value=str(
                    planner_share / "config" / "approach_execution.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "task_orchestration_config",
                default_value=str(
                    planner_share / "config" / "task_orchestration.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "enable_reobservation_motion",
                default_value="false",
                description="Allow explicitly armed Nav2 viewpoint movement",
            ),
            DeclareLaunchArgument(
                "enable_push_cycle_motion",
                default_value="false",
                description="Allow explicitly armed approach, push and return motion",
            ),
            DeclareLaunchArgument(
                "fastdds_builtin_transports",
                default_value="UDPv4",
                description=(
                    "Fast DDS builtin transport set. UDPv4 avoids stale SHM "
                    "port-lock failures on the robot."
                ),
            ),
            # This must execute before any Node action creates a DDS
            # participant.  Keep it scoped to this launch instead of changing
            # the user's global shell environment, because other robot tasks
            # may still benefit from same-host shared-memory transport.
            SetEnvironmentVariable(
                "FASTDDS_BUILTIN_TRANSPORTS",
                LaunchConfiguration("fastdds_builtin_transports"),
            ),
            # Start the consumer first. Transient-local QoS also makes startup
            # order harmless if process scheduling changes this order.
            Node(
                package="cylinder_push_planner",
                executable="selection_planner",
                name="cylinder_push_selection_planner",
                output="screen",
                parameters=[LaunchConfiguration("selection_config")],
            ),
            Node(
                package="cylinder_push_planner",
                executable="reobservation",
                name="cylinder_reobservation",
                output="screen",
                parameters=[
                    LaunchConfiguration("reobservation_config"),
                    {
                        "execution_enabled": ParameterValue(
                            LaunchConfiguration("enable_reobservation_motion"),
                            value_type=bool,
                        )
                    },
                ],
            ),
            Node(
                package="cylinder_push_planner",
                executable="push_cycle_execution",
                name="cylinder_push_approach_execution",
                output="screen",
                parameters=[
                    LaunchConfiguration("approach_execution_config"),
                    {
                        "execution_enabled": ParameterValue(
                            LaunchConfiguration("enable_push_cycle_motion"),
                            value_type=bool,
                        )
                    },
                ],
            ),
            Node(
                package="cylinder_push_planner",
                executable="task_orchestrator",
                name="cylinder_task_orchestrator",
                output="screen",
                parameters=[LaunchConfiguration("task_orchestration_config")],
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
                package="color_object_sorter",
                executable="candidate_validator",
                name="cylinder_candidate_validator",
                output="screen",
                parameters=[LaunchConfiguration("perception_config")],
            ),
        ]
    )
