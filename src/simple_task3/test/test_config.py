"""Test deployed simple controller configuration invariants."""

from pathlib import Path

import yaml


def test_motion_is_disabled_and_path_only_mode_uses_map() -> None:
    config = Path(__file__).parents[1] / "config" / "task3_simple.yaml"
    parameters = yaml.safe_load(config.read_text(encoding="utf-8"))[
        "simple_task_controller"
    ]["ros__parameters"]

    assert parameters["execution_enabled"] is False
    assert parameters["path_only_mode"] is True
    assert parameters["fixed_frame"] == "map"
    assert parameters["live_obstacle_stop_enabled"] is False
    assert parameters["cmd_vel_topic"] == "/cmd_vel"
    assert parameters["require_fixed_localization"] is True
    assert parameters["localization_status_topic"] == "/simple_task3/localization_status"


def test_path_only_mode_keeps_planning_clearance_enabled() -> None:
    source = (
        Path(__file__).parents[1] / "simple_task3" / "controller_node.py"
    ).read_text(encoding="utf-8")

    assert "require_clear_corridors=True" in source
    assert 'self._abort("laser scan became stale")' not in source
    assert 'self._abort("sensor timestamp moved backwards")' not in source
    assert "from std_srvs.srv import Trigger" in source
    assert "self._map_update_step(True)" not in source
    assert '"fixed_localization_ready": self._fixed_localization_ready' in source


def test_localization_handoff_does_not_add_a_motion_stop_condition() -> None:
    source = (
        Path(__file__).parents[1] / "simple_task3" / "controller_node.py"
    ).read_text(encoding="utf-8")

    tick_body = source.split("    def _tick(self)", 1)[1].split(
        "    def _tick_automation", 1
    )[0]
    assert "fixed_localization" not in tick_body


def test_launch_contains_amcl_and_handoff() -> None:
    launch = (Path(__file__).parents[1] / "launch" / "task3_simple.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'package="nav2_amcl"' in launch
    assert 'package="nav2_lifecycle_manager"' in launch
    assert 'executable="localization_handoff"' in launch
    assert '("map", "/task3/fixed_map")' in launch


def test_competition_inventory_is_two_of_each_color() -> None:
    config = Path(__file__).parents[1] / "config" / "task3_simple.yaml"
    parameters = yaml.safe_load(config.read_text(encoding="utf-8"))[
        "simple_task_controller"
    ]["ros__parameters"]

    assert parameters["inventory_colors"] == ["blue", "green", "red"]
    assert parameters["inventory_counts"] == [2, 2, 2]
