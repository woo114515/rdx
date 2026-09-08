import math

import pytest

from cylinder_push_planner.direct_cycle import (
    build_direct_cycle,
    distance_to_segment,
    enforce_minimum_linear_speed,
    heading_error,
    polyline_tracking_target,
    pure_pursuit_command,
    transform_point_2d,
)
from cylinder_push_planner.geometry import Target, select_right_first


def target(candidate_id, x, y, color="blue"):
    return Target(candidate_id, x, y, color, 0.035)


def test_direct_cycle_places_robot_behind_target_and_destination() -> None:
    targets = (target(1, 1.0, -0.3), target(2, 1.0, 0.3))
    selection = select_right_first(targets, 0.0, 0.0, 0.25)
    plan = build_direct_cycle(
        targets,
        (0.0, 0.0, 0.0),
        (2.0, -0.3),
        0.25,
        0.22,
        0.15,
        0.10,
        selection=selection,
        require_clear_corridors=False,
    )

    assert plan.selection.target.candidate_id == 1
    assert plan.contact == pytest.approx((0.78, -0.3))
    assert plan.staging == pytest.approx((0.53, -0.3))
    assert plan.push_end == pytest.approx((1.78, -0.3))
    assert plan.release_end == pytest.approx((1.63, -0.3))
    expected_return = (
        (1.63, -0.3),
        (0.78, -0.3),
        (0.53, -0.3),
        (0.0, 0.0),
    )
    for actual, expected in zip(plan.return_path, expected_return):
        assert actual == pytest.approx(expected)
    assert plan.return_path[-1] == pytest.approx((0.0, 0.0))


def test_direct_cycle_routes_around_remaining_cylinder_keepout() -> None:
    selected = target(1, 1.0, 0.0)
    blocker = target(2, 1.5, 0.0, "red")
    selection = select_right_first((selected,), 0.0, 0.0, 0.25)

    plan = build_direct_cycle(
        (selected, blocker),
        (0.0, -0.5, 0.0),
        (2.0, 0.0),
        0.25,
        0.22,
        0.15,
        0.10,
        selection=selection,
    )

    minimum_clearance = 0.10 + blocker.radius
    assert all(
        math.dist(point, (blocker.x, blocker.y)) >= minimum_clearance - 1e-6
        for point in plan.push_path
    )
    assert any(abs(point[1]) > 0.01 for point in plan.push_path)
    assert plan.keepout_boundary


def test_distance_to_segment_clamps_to_endpoints() -> None:
    assert distance_to_segment((0.5, 1.0), (0.0, 0.0), (1.0, 0.0)) == 1.0
    assert distance_to_segment((2.0, 0.0), (0.0, 0.0), (1.0, 0.0)) == 1.0


def test_polyline_tracking_target_uses_projection_and_lookahead() -> None:
    target_point, deviation = polyline_tracking_target(
        ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0)),
        (0.90, 0.04),
        0.20,
    )

    assert deviation == pytest.approx(0.04)
    assert target_point == pytest.approx((1.0, 0.10))


def test_polyline_tracking_target_rejects_invalid_input() -> None:
    with pytest.raises(ValueError, match="at least two"):
        polyline_tracking_target(((0.0, 0.0),), (0.0, 0.0), 0.15)
    with pytest.raises(ValueError, match="positive and finite"):
        polyline_tracking_target(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0), 0.0)


def test_pure_pursuit_command_is_symmetric_and_curvature_limited() -> None:
    left = pure_pursuit_command(0.30, 0.15, 0.20, 0.30, 0.60, 0.50)
    right = pure_pursuit_command(0.30, 0.15, -0.20, 0.30, 0.60, 0.50)

    assert left[0] == pytest.approx(right[0])
    assert left[1] == pytest.approx(-right[1])
    assert 0.15 <= left[0] <= 0.30
    assert 0.0 < left[1] <= 0.50


def test_pure_pursuit_rotates_before_an_untrackable_turn() -> None:
    linear, angular = pure_pursuit_command(
        0.30, 0.15, 0.59, 0.10, 0.60, 0.50
    )

    assert linear == 0.0
    assert angular == pytest.approx(0.50)


def test_return_retraces_clear_segments_instead_of_cutting_across_field() -> None:
    selected = target(1, 1.0, 0.0)
    # This cylinder lies on the old direct delivery-to-home chord, but not on
    # the outbound approach/contact/push corridors that are now retraced.
    blocker = target(2, 0.8, -0.5, "red")
    selection = select_right_first((selected,), 0.0, -1.0, 0.25)

    plan = build_direct_cycle(
        (selected, blocker),
        (0.0, -1.0, 0.0),
        (2.0, 0.0),
        0.25,
        0.15,
        0.15,
        0.10,
        selection=selection,
    )

    assert distance_to_segment(
        (blocker.x, blocker.y), plan.release_end, plan.home[:2]
    ) < 0.135
    assert plan.return_path[-1] == plan.home[:2]


def test_heading_error_wraps_at_pi() -> None:
    assert heading_error(math.radians(179), math.radians(-179)) == pytest.approx(
        math.radians(2)
    )


def test_transform_point_2d_applies_translation_and_rotation() -> None:
    transformed = transform_point_2d(
        (1.0, 0.0),
        (2.0, -1.0),
        math.pi / 2.0,
    )

    assert transformed == pytest.approx((2.0, 0.0))


@pytest.mark.parametrize(
    ("command", "expected"),
    [
        (0.0, 0.0),
        (0.05, 0.15),
        (-0.05, -0.15),
        (0.20, 0.20),
        (-0.20, -0.20),
    ],
)
def test_minimum_linear_speed_preserves_stops_and_direction(
    command: float, expected: float
) -> None:
    assert enforce_minimum_linear_speed(command, 0.15) == pytest.approx(expected)


def test_minimum_linear_speed_rejects_invalid_threshold() -> None:
    with pytest.raises(ValueError, match="must not be negative"):
        enforce_minimum_linear_speed(0.1, -0.1)
