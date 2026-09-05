from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("rdx_color_sorting"))
    default_config = str(package_share / "config" / "color_sorting.yaml")
    config_argument = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to the color sorting parameter file.",
    )
    sorting_node = Node(
        package="rdx_color_sorting",
        executable="color_sorting_node",
        name="color_sorting",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )
    return LaunchDescription([config_argument, sorting_node])
