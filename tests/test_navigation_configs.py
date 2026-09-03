from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SLAM_CONFIG = ROOT / "src/rdx_navigation/config/slam_toolbox.yaml"
NAV2_CONFIG = ROOT / "src/rdx_navigation/config/nav2.yaml"
MAPPING_LAUNCH = ROOT / "src/rdx_navigation/launch/mapping.launch.py"
NAVIGATION_LAUNCH = ROOT / "src/rdx_navigation/launch/navigation.launch.py"


def load_ros_params(path: Path, node_name: str):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return data[node_name]["ros__parameters"]


def test_slam_uses_real_time_and_verified_frames():
    params = load_ros_params(SLAM_CONFIG, "slam_toolbox")

    assert params["use_sim_time"] is False
    assert params["scan_topic"] == "/scan"
    assert params["odom_frame"] == "odom"
    assert params["base_frame"] == "base_footprint"
    assert params["map_frame"] == "map"
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.05
    assert params["scan_queue_size"] == 1
    assert params["minimum_travel_distance"] == 0.15
    assert params["minimum_travel_heading"] == 0.15


def test_mapping_launch_uses_async_slam_and_teleop_input():
    source = MAPPING_LAUNCH.read_text(encoding="utf-8")

    assert "online_async_launch.py" in source
    assert "cmd_vel_teleop" in source
    assert "start_hardware" in source
    assert "map_saver_cli" not in source


def test_nav2_is_real_robot_and_routes_velocity_through_safety():
    data = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))

    assert all(
        value is False
        for node in data.values()
        if isinstance(node, dict)
        for value in [node.get("ros__parameters", {}).get("use_sim_time", False)]
    )
    assert data["bt_navigator"]["ros__parameters"]["odom_topic"] == "/odom_raw"
    follow = data["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["max_vel_x"] <= 0.18
    assert follow["max_vel_y"] <= 0.18
    assert follow["max_vel_theta"] <= 0.60


def test_costmaps_use_scan_and_conservative_clearance():
    data = yaml.safe_load(NAV2_CONFIG.read_text(encoding="utf-8"))
    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = data["global_costmap"]["global_costmap"]["ros__parameters"]

    assert local["inflation_layer"]["inflation_radius"] >= 0.55
    assert global_["inflation_layer"]["inflation_radius"] >= 0.55
    assert local["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert global_["obstacle_layer"]["scan"]["topic"] == "/scan"


def test_navigation_launch_requires_map_and_routes_cmd_vel_nav():
    source = NAVIGATION_LAUNCH.read_text(encoding="utf-8")

    assert "bringup_launch.py" in source
    assert "cmd_vel_nav" in source
    assert "map" in source
    assert "footprint_verified" in source
    assert "RewrittenYaml" in source
    assert "/opt/ros/humble/share/rdx_navigation" not in (
        NAV2_CONFIG.read_text(encoding="utf-8")
    )
