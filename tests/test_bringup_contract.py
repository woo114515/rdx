from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SAFETY_CONFIG = ROOT / "src/rdx_safety/config/safety.yaml"
HARDWARE_LAUNCH = ROOT / "src/rdx_bringup/launch/hardware.launch.py"


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
    assert params["output_topic"] == "/cmd_vel"
    assert params["nav_topic"] == "/cmd_vel_nav"
    assert params["teleop_topic"] == "/cmd_vel_teleop"


def test_hardware_launch_is_minimal_and_has_no_teleop_source():
    source = HARDWARE_LAUNCH.read_text(encoding="utf-8")

    assert "yahboomcar_base_node" in source
    assert "Mcnamu_driver" in source
    assert "yahboomcar_description" in source
    assert "oradar_lidar_ms200" in source
    assert "rdx_safety_node" in source
    assert "joy" not in source.lower()
    assert "keyboard" not in source.lower()
    assert '"/MS200/scan", "/scan"' in source
