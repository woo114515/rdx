"""Start the motion-free Task 3 perception and planning pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    perception_share = Path(get_package_share_directory("color_object_sorter"))
    snapshot_share = Path(get_package_share_directory("cylinder_field_mapping"))

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
            # Start the consumer first. Transient-local QoS also makes startup
            # order harmless if process scheduling changes this order.
            Node(
                package="cylinder_push_planner",
                executable="selection_planner",
                name="cylinder_push_selection_planner",
                output="screen",
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
