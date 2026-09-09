import math

import pytest

from cylinder_push_planner.geometry import (
    Target,
    build_push_preview,
    build_remaining_keepout,
    convex_hull,
    destination_slot,
    generate_reobservation_viewpoints,
    local_offset_to_map,
    open_destination_slot,
    path_length,
    path_stays_outside,
    point_sets_stable,
    points_near_reference,
    point_outside_keepout,
    polyline_clear_of_targets,
    polyline_outside_keepout,
    select_right_first,
)


def test_destination_offset_uses_fixed_field_center_and_task_axes() -> None:
    assert local_offset_to_map(2.0, -0.1, 0.0, 1.5, -0.09) == pytest.approx(
        (3.5, -0.19)
    )
    assert local_offset_to_map(
        2.0, -0.1, math.pi / 2.0, 1.5, -0.09
    ) == pytest.approx((2.09, 1.4))


def test_destination_transform_rejects_non_finite_values() -> None:
    with pytest.raises(ValueError, match="must be finite"):
        local_offset_to_map(0.0, 0.0, 0.0, math.nan, 0.0)


def test_snapshot_stability_is_order_independent_and_rejects_motion() -> None:
    previous = ((1.0, 0.0), (1.3, 0.1))
    assert point_sets_stable(previous, ((1.32, 0.11), (0.98, 0.01)), 0.05)
    assert not point_sets_stable(previous, ((1.4, 0.1), (1.0, 0.0)), 0.05)
    assert not point_sets_stable(previous, ((1.0, 0.0),), 0.05)


def test_new_candidates_must_remain_near_the_accepted_field() -> None:
    reference = ((1.0, 0.0), (1.3, 0.0))
    assert points_near_reference(((1.02, 0.01), (1.58, 0.0)), reference, 0.30)
    assert not points_near_reference(((1.02, 0.01), (2.0, 0.0)), reference, 0.30)


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


def test_reobservation_viewpoints_stay_outside_and_face_center() -> None:
    targets = (
        Target(1, 2.0, 0.3, "", 0.035),
        Target(2, 2.0, -0.3, "", 0.035),
        Target(3, 2.4, 0.0, "", 0.035),
    )
    right, left = generate_reobservation_viewpoints(
        targets, 0.0, 0.0, 0.35, 0.30, 0.10, math.radians(25.0)
    )
    assert right.side == "right"
    assert left.side == "left"
    for viewpoint in (right, left):
        distance = math.hypot(
            viewpoint.x - viewpoint.envelope_x,
            viewpoint.y - viewpoint.envelope_y,
        )
        assert distance >= viewpoint.minimum_center_radius
        expected_yaw = math.atan2(
            viewpoint.envelope_y - viewpoint.y,
            viewpoint.envelope_x - viewpoint.x,
        )
        assert viewpoint.yaw == pytest.approx(expected_yaw)
    assert right.y < left.y


def test_path_metrics_reject_a_route_through_group() -> None:
    outside = ((0.0, 0.0), (0.0, 0.3), (0.1, 0.6))
    assert path_length(outside) == pytest.approx(0.3 + math.hypot(0.1, 0.3))
    assert path_stays_outside(outside, 1.0, 0.0, 0.5)
    assert not path_stays_outside(((0.0, 0.0), (0.8, 0.0)), 1.0, 0.0, 0.5)


def test_destination_slots_are_centered_and_extensible() -> None:
    first = destination_slot(1.5, 0.0, 0, 2, 0.18)
    second = destination_slot(1.5, 0.0, 1, 2, 0.18)
    assert first == pytest.approx((1.5, -0.09))
    assert second == pytest.approx((1.5, 0.09))
    assert math.dist(first, second) == pytest.approx(0.18)
    assert destination_slot(1.2, 0.9, 1, 3, 0.18) == pytest.approx(
        (1.2, 0.9)
    )
    with pytest.raises(ValueError):
        destination_slot(1.0, 0.0, 2, 2, 0.18)


def test_open_destination_slots_preserve_nominal_pair_and_extend_outward() -> None:
    slots = tuple(open_destination_slot(1.5, 0.0, i, 2, 0.18) for i in range(4))
    expected = ((1.5, -0.09), (1.5, 0.09), (1.5, -0.27), (1.5, 0.27))
    for actual, wanted in zip(slots, expected):
        assert actual == pytest.approx(wanted)


def test_open_destination_accepts_color_with_zero_inventory_hint() -> None:
    assert open_destination_slot(1.5, 0.0, 0, 0, 0.18) == pytest.approx(
        (1.5, 0.0)
    )


def test_convex_hull_discards_interior_points() -> None:
    hull = convex_hull(
        ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.5, 0.5))
    )
    assert hull == ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))


def test_remaining_keepout_excludes_only_selected_target() -> None:
    targets = (
        Target(1, 0.0, 0.0, "red", 0.035),
        Target(2, 1.0, 0.0, "blue", 0.035),
        Target(3, 1.0, 1.0, "green", 0.035),
        Target(4, 0.0, 1.0, "red", 0.035),
        Target(5, 0.5, 0.5, "blue", 0.035),
        Target(6, -0.4, 0.5, "green", 0.035),
    )
    keepout = build_remaining_keepout(targets, 6, 0.20)
    assert (-0.4, 0.5) not in keepout.hull
    assert not point_outside_keepout((0.5, 0.5), keepout)
    assert point_outside_keepout((-0.4, 0.5), keepout)
    with pytest.raises(ValueError):
        build_remaining_keepout(targets, 99, 0.20)


def test_polyline_detects_crossing_remaining_keepout() -> None:
    targets = (
        Target(1, 0.0, 0.0, "red", 0.035),
        Target(2, 1.0, 0.0, "blue", 0.035),
        Target(3, 1.0, 1.0, "green", 0.035),
        Target(4, 0.0, 1.0, "red", 0.035),
    )
    keepout = build_remaining_keepout(targets, 1, 0.10)
    assert polyline_outside_keepout(((-0.5, -0.5), (-0.2, -0.2)), keepout)
    assert not polyline_outside_keepout(((-0.5, -0.5), (1.2, 1.2)), keepout)


def test_local_path_inside_hull_can_still_clear_each_cylinder() -> None:
    remaining = (
        Target(1, -1.0, 0.0, "red", 0.035),
        Target(2, 1.0, 0.0, "blue", 0.035),
        Target(3, 0.0, 1.5, "green", 0.035),
    )
    keepout = build_remaining_keepout(
        (*remaining, Target(4, 0.0, -1.0, "red", 0.035)),
        4,
        0.135,
    )
    path = ((0.0, 0.4), (0.0, -0.5))

    assert not polyline_outside_keepout(path, keepout)
    assert polyline_clear_of_targets(path, remaining, 0.135)


def test_push_preview_excludes_target_and_keeps_paths_outside() -> None:
    targets = (
        Target(1, 2.0, -0.6, "red", 0.035),
        Target(2, 2.0, 0.0, "blue", 0.035),
        Target(3, 2.0, 0.6, "green", 0.035),
        Target(4, 2.5, -0.6, "red", 0.035),
        Target(5, 2.5, 0.0, "blue", 0.035),
        Target(6, 2.5, 0.6, "green", 0.035),
    )
    plan = build_push_preview(
        targets,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        destination_x=1.2,
        destination_y=-0.9,
        staging_clearance=0.25,
        robot_clearance=0.21,
        contact_offset=0.22,
        release_retreat=0.15,
    )
    assert plan.selection.target.candidate_id == 1
    assert (2.0, -0.6) not in plan.keepout.hull
    assert polyline_outside_keepout(plan.approach_path, plan.keepout)
    assert polyline_outside_keepout(plan.robot_push_path, plan.keepout)
    assert polyline_outside_keepout(plan.return_path, plan.keepout)
    assert plan.target_path[0] == pytest.approx((2.0, -0.6))
    assert plan.approach_path[0] == pytest.approx((0.0, 0.0))
    assert plan.approach_path[-1] == pytest.approx(
        (plan.staging_x, plan.staging_y)
    )
    assert math.dist(
        (plan.staging_x, plan.staging_y), plan.robot_push_path[0]
    ) == pytest.approx(0.25)
    assert plan.target_path[-1] == pytest.approx((1.2, -0.9))
    assert plan.return_path[-1] == pytest.approx((0.0, 0.0))


def test_push_preview_rejects_destination_inside_initial_envelope() -> None:
    targets = (
        Target(1, 2.0, -0.6, "red", 0.035),
        Target(2, 2.0, 0.0, "blue", 0.035),
        Target(3, 2.0, 0.6, "green", 0.035),
        Target(4, 2.5, -0.6, "red", 0.035),
        Target(5, 2.5, 0.0, "blue", 0.035),
        Target(6, 2.5, 0.6, "green", 0.035),
    )
    with pytest.raises(ValueError, match="outside the initial cylinder envelope"):
        build_push_preview(
            targets,
            0.0,
            0.0,
            0.0,
            2.3,
            0.0,
            0.25,
            0.21,
            0.22,
            0.15,
        )


def test_push_preview_rejects_delivery_exclusion_over_remaining_target() -> None:
    targets = (
        Target(1, 1.0, -0.5, "red", 0.035),
        Target(2, 1.0, 0.0, "blue", 0.035),
        Target(3, 1.0, 0.5, "green", 0.035),
    )
    selection = select_right_first(targets, 0.0, 0.0, 0.25)
    with pytest.raises(ValueError, match="exclusion zone overlaps"):
        build_push_preview(
            targets,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            destination_x=1.0,
            destination_y=0.70,
            staging_clearance=0.25,
            robot_clearance=0.21,
            contact_offset=0.22,
            release_retreat=0.15,
            envelope_exit_clearance=0.0,
            selection=selection,
            delivered_exclusion_radius=0.18,
        )


def test_last_cylinder_preview_has_valid_empty_keepout() -> None:
    plan = build_push_preview(
        (Target(1, 1.0, 0.0, "red", 0.035),),
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        destination_x=1.2,
        destination_y=-0.9,
        staging_clearance=0.25,
        robot_clearance=0.21,
        contact_offset=0.22,
        release_retreat=0.15,
        delivered_exclusion_radius=0.18,
    )

    assert plan.keepout.hull == ()
    assert plan.keepout.boundary == ()
    assert polyline_outside_keepout(plan.approach_path, plan.keepout)
    assert polyline_outside_keepout(plan.robot_push_path, plan.keepout)
    assert polyline_outside_keepout(plan.return_path, plan.keepout)


def test_preview_can_return_to_fixed_task_home_from_drifted_start() -> None:
    targets = (
        Target(1, 2.0, -0.5, "red", 0.035),
        Target(2, 2.0, 0.0, "blue", 0.035),
        Target(3, 2.0, 0.5, "green", 0.035),
    )
    plan = build_push_preview(
        targets,
        robot_x=0.08,
        robot_y=-0.04,
        robot_yaw=0.03,
        destination_x=1.2,
        destination_y=-0.9,
        staging_clearance=0.25,
        robot_clearance=0.21,
        contact_offset=0.22,
        release_retreat=0.15,
        task_home=(0.0, 0.0, 0.0),
    )

    assert plan.approach_path[0] == pytest.approx((0.08, -0.04))
    assert plan.return_path[-1] == pytest.approx((0.0, 0.0))
    assert (plan.home_x, plan.home_y, plan.home_yaw) == (0.0, 0.0, 0.0)


def test_push_preview_finds_wide_detour_for_recorded_field_layout() -> None:
    """Regression for the 2026-09-07 six-cylinder static field."""

    targets = (
        Target(1, 1.4959, 0.1654, "blue", 0.035),
        Target(2, 1.7373, 0.3599, "green", 0.035),
        Target(3, 1.0824, 0.3299, "green", 0.035),
        Target(7, 1.7658, -0.4559, "blue", 0.035),
        Target(8, 1.4316, -0.1531, "red", 0.035),
        Target(9, 0.9881, -0.0123, "red", 0.035),
    )
    plan = build_push_preview(
        targets,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        destination_x=1.2,
        destination_y=0.9,
        staging_clearance=0.25,
        robot_clearance=0.21,
        contact_offset=0.22,
        release_retreat=0.15,
    )
    assert plan.selection.target.candidate_id == 7
    assert all(
        polyline_outside_keepout(path, plan.keepout)
        for path in (
            plan.approach_path,
            plan.robot_push_path,
            plan.return_path,
        )
    )
