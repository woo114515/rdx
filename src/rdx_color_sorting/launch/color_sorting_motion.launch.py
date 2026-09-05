from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("rdx_color_sorting"))
    default_config = str(package_share / "config" / "color_sorting.yaml")
    arguments = [
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("output_enabled", default_value="false"),
        DeclareLaunchArgument("pose_source", default_value="odom"),
        DeclareLaunchArgument(
            "zones_relative_to_origin",
            default_value="true",
        ),
        DeclareLaunchArgument("nav2_return_enabled", default_value="false"),
        DeclareLaunchArgument(
            "navigation_request_enabled",
            default_value="false",
        ),
        DeclareLaunchArgument("lidar_stop_enabled", default_value="false"),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/csi/image_raw/compressed",
        ),
        DeclareLaunchArgument("compressed_input", default_value="true"),
    ]
    sorting_node = Node(
        package="rdx_color_sorting",
        executable="color_sorting_node",
        name="color_sorting",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "motion_enabled": LaunchConfiguration("motion_enabled"),
                "image_topic": LaunchConfiguration("image_topic"),
                "compressed_input": LaunchConfiguration("compressed_input"),
                "pose_source": LaunchConfiguration("pose_source"),
                "zones_relative_to_origin": LaunchConfiguration(
                    "zones_relative_to_origin"
                ),
                "nav2_return_enabled": LaunchConfiguration(
                    "nav2_return_enabled"
                ),
            },
        ],
    )
    safety_node = Node(
        package="rdx_color_sorting",
        executable="safety_arbiter_node",
        name="rdx_sorting_safety",
        output="screen",
        parameters=[
            LaunchConfiguration("config"),
            {
                "output_enabled": LaunchConfiguration("output_enabled"),
                "navigation_request_enabled": LaunchConfiguration(
                    "navigation_request_enabled"
                ),
                "lidar_stop_enabled": LaunchConfiguration(
                    "lidar_stop_enabled"
                ),
            },
        ],
    )
    return LaunchDescription(arguments + [sorting_node, safety_node])
