from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SAFETY_CONFIG = ROOT / "src/rdx_safety/config/safety.yaml"
HARDWARE_LAUNCH = ROOT / "src/rdx_bringup/launch/hardware.launch.py"
BRINGUP_PACKAGE = ROOT / "src/rdx_bringup/package.xml"


def test_safety_configuration_fails_closed():
    data = yaml.safe_load(SAFETY_CONFIG.read_text(encoding="utf-8"))
    params = data["rdx_safety"]["ros__parameters"]

    assert params["use_sim_time"] is False
    assert params["footprint_verified"] is False
    assert params["emergency_stop_on_start"] is True
    assert params["command_timeout"] <= 0.25
    assert params["scan_timeout"] <= 0.30
    assert params["max_linear_speed"] <= 0.18
    assert params["max_angular_speed"] <= 0.60
    assert params["output_topic"] == "/cmd_vel_safe"
    assert params["nav_topic"] == "/cmd_vel_nav"
    assert params["teleop_topic"] == "/cmd_vel_teleop"


def test_hardware_launch_is_minimal_and_has_no_teleop_source():
    source = HARDWARE_LAUNCH.read_text(encoding="utf-8")

    assert "yahboomcar_base_node" in source
    assert "Mcnamu_driver" in source
    assert "yahboomcar_description" in source
    assert 'package="oradar_lidar"' in source
    assert 'executable="oradar_scan"' in source
    assert "oradar_lidar_ms200" not in source
    assert "respawn=True" in source
    assert "respawn_delay=2.0" in source
    assert "rdx_safety_node" in source
    assert "joy" not in source.lower()
    assert "keyboard" not in source.lower()
    assert 'remappings=[("scan", "/scan")]' in source
    assert '"motion_command_topic": "/cmd_vel_safe"' in source


def test_bringup_declares_the_robot_installed_lidar_package():
    package_xml = BRINGUP_PACKAGE.read_text(encoding="utf-8")

    assert "<exec_depend>oradar_lidar</exec_depend>" in package_xml
    assert "oradar_lidar_ms200" not in package_xml
