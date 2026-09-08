import math

import pytest

from cylinder_push_planner.execution_safety import (
    approach_watchdog_fault,
    closest_path_index,
    front_target_present,
    nav2_path_result_error_code,
    obstacle_behind,
    path_stays_outside_polygon,
    plan_inputs_fault,
    point_outside_polygon,
    retreat_motion,
    retreat_pose_step,
    tracking_command,
    unexpected_obstacle_ahead,
)


def test_nav2_path_result_error_code_supports_humble_and_newer_interfaces() -> None:
    class HumbleResult:
        pass

    class NewerResult:
        error_code = 7

    assert nav2_path_result_error_code(HumbleResult()) == 0
    assert nav2_path_result_error_code(NewerResult()) == 7


def test_path_outside_polygon_accepts_clear_detour() -> None:
    polygon = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
    assert path_stays_outside_polygon(((-0.5, -0.2), (1.5, -0.2)), polygon)


def test_path_outside_polygon_rejects_crossing_between_sparse_poses() -> None:
    polygon = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
    assert not path_stays_outside_polygon(((-0.5, 0.5), (1.5, 0.5)), polygon)
    assert point_outside_polygon((-0.1, 0.5), polygon)
    assert not point_outside_polygon((0.5, 0.5), polygon)
    assert not point_outside_polygon((0.0, 0.5), polygon)


def test_path_outside_polygon_rejects_invalid_inputs() -> None:
    assert not path_stays_outside_polygon(((0.0, 0.0),), ((0.0, 0.0),) * 3)
    assert path_stays_outside_polygon(((0.0, 0.0), (1.0, 0.0)), ())
    assert point_outside_polygon((0.0, 0.0), ())
    assert not path_stays_outside_polygon(
        ((0.0, 0.0), (1.0, 0.0)), ((0.0, 0.0), (1.0, 0.0))
    )
    with pytest.raises(ValueError):
        path_stays_outside_polygon(((0.0, 0.0), (1.0, 0.0)), ((0.0, 0.0),) * 3, 0.0)


def test_plan_inputs_report_first_missing_gate() -> None:
    assert plan_inputs_fault(False, True, True, True, True, True) == (
        "push preview is not ready"
    )
    assert plan_inputs_fault(True, True, True, True, True, True) is None
    assert "push path" in plan_inputs_fault(
        True, True, True, True, True, True, False, True
    )
    assert "return path" in plan_inputs_fault(
        True, True, True, True, True, True, True, False
    )
    assert "target path" in plan_inputs_fault(
        True, True, True, True, True, True, True, True, False
    )


def test_approach_watchdog_faults_and_clear_state() -> None:
    assert "missing" in approach_watchdog_fault(10.0, 0.0, 9.0, 1.0, 20.0)
    assert "stale" in approach_watchdog_fault(10.0, 8.0, 9.0, 1.0, 20.0)
    assert "timed out" in approach_watchdog_fault(31.0, 30.9, 10.0, 1.0, 20.0)
    assert approach_watchdog_fault(10.0, 9.9, 9.0, 1.0, 20.0) is None


def test_tracking_command_is_forward_only_and_progress_is_monotonic() -> None:
    path = ((0.0, 0.0), (0.2, 0.0), (0.4, 0.1), (0.6, 0.2))
    assert closest_path_index(path, (0.21, 0.01), 0) == 1
    linear, angular, progress, error = tracking_command(
        path, 0.21, 0.01, 0.0, 1, 0.12, 0.06, 1.5, 0.25
    )
    assert 0.0 <= linear <= 0.06
    assert 0.0 < angular <= 0.25
    assert progress >= 1
    assert error > 0.0
    assert closest_path_index(path, (0.0, 0.0), 2) == 2


def test_retreat_motion_uses_initial_robot_axes() -> None:
    progress, lateral, heading = retreat_motion(
        1.0,
        2.0,
        math.pi / 2.0,
        1.0,
        1.85,
        math.pi / 2.0,
    )
    assert progress == pytest.approx(0.15)
    assert lateral == pytest.approx(0.0, abs=1e-9)
    assert heading == pytest.approx(0.0)


def test_retreat_motion_reports_sideways_and_heading_errors() -> None:
    progress, lateral, heading = retreat_motion(
        0.0,
        0.0,
        0.0,
        -0.12,
        0.04,
        0.10,
    )
    assert progress == pytest.approx(0.12)
    assert lateral == pytest.approx(0.04)
    assert heading == pytest.approx(0.10)


def test_retreat_motion_reports_forward_motion_as_negative_progress() -> None:
    progress, _lateral, _heading = retreat_motion(
        0.0,
        0.0,
        0.0,
        0.03,
        0.0,
        0.0,
    )
    assert progress == pytest.approx(-0.03)


def test_retreat_pose_step_detects_localization_jump() -> None:
    assert retreat_pose_step(0.0, 0.0, -0.004, 0.001) < 0.01
    assert retreat_pose_step(0.0, 0.0, -0.15, 0.0) == pytest.approx(0.15)


def test_retreat_helpers_reject_non_finite_pose() -> None:
    with pytest.raises(ValueError):
        retreat_pose_step(0.0, 0.0, float("nan"), 0.0)
    with pytest.raises(ValueError):
        retreat_motion(0.0, 0.0, 0.0, float("inf"), 0.0, 0.0)


def test_front_target_and_side_obstacle_are_distinguished() -> None:
    # Three samples at -0.2, 0.0 and +0.2 rad. The centre return is the target.
    ranges = (0.25, 0.20, float("inf"))
    assert front_target_present(ranges, -0.2, 0.2, 0.15, 20.0, 0.30, 0.06)
    assert not unexpected_obstacle_ahead(
        ranges, -0.2, 0.2, 0.15, 20.0, 0.30, 0.20, 0.06, 0.30
    )
    # A close off-centre return is not hidden by the target corridor.
    assert unexpected_obstacle_ahead(
        (0.18, 0.20, float("inf")),
        -0.5,
        0.5,
        0.15,
        20.0,
        0.30,
        0.20,
        0.06,
        0.30,
    )


def test_reverse_obstacle_uses_rear_half_plane() -> None:
    # Angles 0 and pi: only the second point is behind the robot.
    assert obstacle_behind(
        (0.20, 0.18), 0.0, math.pi, 0.1, 20.0, 0.25, 0.15
    )
    assert not obstacle_behind((0.20,), 0.0, 1.0, 0.1, 20.0, 0.25, 0.15)
