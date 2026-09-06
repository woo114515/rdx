import math

import pytest

from cylinder_push_planner.geometry import Target, select_right_first


def test_selects_rightmost_target_from_robot_view() -> None:
    targets = (
        Target(1, 2.0, 0.5, "red", 0.035),
        Target(2, 2.0, -0.5, "blue", 0.035),
        Target(3, 2.5, 0.0, "green", 0.035),
    )
    plan = select_right_first(targets, 0.0, 0.0, 0.25)
    assert plan.target.candidate_id == 2


def test_staging_point_is_outside_envelope_and_faces_target() -> None:
    targets = (
        Target(1, 2.0, 0.4, "red", 0.035),
        Target(2, 2.0, -0.4, "blue", 0.035),
    )
    plan = select_right_first(targets, 0.0, 0.0, 0.25)
    distance = math.hypot(
        plan.staging_x - plan.envelope_x,
        plan.staging_y - plan.envelope_y,
    )
    assert distance == pytest.approx(plan.envelope_radius + 0.25)
    expected_yaw = math.atan2(
        plan.target.y - plan.staging_y,
        plan.target.x - plan.staging_x,
    )
    assert plan.staging_yaw == pytest.approx(expected_yaw)


def test_rejects_empty_target_set() -> None:
    with pytest.raises(ValueError):
        select_right_first((), 0.0, 0.0, 0.25)
