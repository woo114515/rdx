import pytest

from cylinder_field_mapping.field_map import (
    CylinderFieldMap,
    Observation,
    minimum_enclosing_circle,
)


def obs(x: float, y: float, stamp: float, color: str = "blue") -> Observation:
    return Observation(x, y, color, 0.9, stamp)


def test_spatial_identity_survives_new_camera_observation() -> None:
    field = CylinderFieldMap(association_distance=0.2)
    first = field.update([obs(1.0, 2.0, 0.0)])
    second = field.update([obs(1.05, 1.98, 1.0)])
    assert len(second) == 1
    assert second[0].cylinder_id == first[0].cylinder_id
    assert second[0].observation_count == 2


def test_two_nearby_observations_do_not_share_one_track_per_frame() -> None:
    field = CylinderFieldMap(association_distance=0.2)
    field.update([obs(0.0, 0.0, 0.0)])
    result = field.update([obs(0.02, 0.0, 1.0), obs(0.10, 0.0, 1.0)])
    assert len(result) == 2


def test_color_uses_multiview_majority_vote() -> None:
    field = CylinderFieldMap(association_distance=0.2)
    field.update([obs(0.0, 0.0, 0.0, "red")])
    field.update([obs(0.0, 0.0, 1.0, "green")])
    result = field.update([obs(0.0, 0.0, 2.0, "green")])
    assert result[0].color == "green"


def test_minimum_circle_for_triangle() -> None:
    circle = minimum_enclosing_circle([(0.0, 0.0), (2.0, 0.0), (1.0, 1.0)])
    assert circle is not None
    assert circle.x == pytest.approx(1.0)
    assert circle.y == pytest.approx(0.0)
    assert circle.radius == pytest.approx(1.0)


def test_envelope_requires_observation_threshold() -> None:
    field = CylinderFieldMap()
    field.update([obs(0.0, 0.0, 0.0), obs(1.0, 0.0, 0.0)])
    assert field.envelope(minimum_observations=2) is None
    field.update([obs(0.0, 0.0, 1.0), obs(1.0, 0.0, 1.0)])
    circle = field.envelope(minimum_observations=2)
    assert circle is not None
    assert circle.radius == pytest.approx(0.5)


def test_clear_removes_tracks_and_restarts_ids() -> None:
    field = CylinderFieldMap()
    field.update([obs(0.0, 0.0, 0.0)])
    field.clear()
    assert field.envelope() is None
    result = field.update([obs(1.0, 0.0, 1.0)])
    assert result[0].cylinder_id == 1


def test_stale_track_is_excluded_from_active_map_and_envelope() -> None:
    field = CylinderFieldMap()
    field.update([obs(0.0, 0.0, 0.0), obs(1.0, 0.0, 0.0, "red")])
    field.update([obs(1.0, 0.0, 3.0, "red")])
    active = field.active(maximum_age=2.0)
    assert [(item.cylinder_id, item.color) for item in active] == [(2, "red")]
    circle = field.envelope(maximum_age=2.0)
    assert circle is not None
    assert circle.x == pytest.approx(1.0)
    assert circle.radius == pytest.approx(0.0)


def test_empty_frame_advances_time_and_expires_all_tracks() -> None:
    field = CylinderFieldMap()
    field.update([obs(0.0, 0.0, 0.0)])
    field.update([], stamp=3.0)
    assert field.active(maximum_age=2.0) == ()


def test_briefly_missing_same_color_target_keeps_id_after_relocation() -> None:
    field = CylinderFieldMap(
        association_distance=0.15,
        relocation_distance=0.75,
        relocation_max_age=3.0,
    )
    first = field.update([obs(0.0, 0.0, 0.0, "green")])
    moved = field.update([obs(0.50, 0.0, 1.0, "green")])
    assert len(moved) == 1
    assert moved[0].cylinder_id == first[0].cylinder_id
    assert moved[0].x == pytest.approx(0.50)


def test_relocation_does_not_steal_other_visible_same_color_target() -> None:
    field = CylinderFieldMap(
        association_distance=0.15,
        relocation_distance=0.75,
        relocation_max_age=3.0,
    )
    first = field.update(
        [obs(0.0, 0.0, 0.0, "green"), obs(1.0, 0.0, 0.0, "green")]
    )
    moved = field.update(
        [obs(0.45, 0.0, 1.0, "green"), obs(1.02, 0.0, 1.0, "green")]
    )
    assert len(moved) == 2
    assert {item.cylinder_id for item in moved} == {
        first[0].cylinder_id,
        first[1].cylinder_id,
    }
    by_id = {item.cylinder_id: item for item in moved}
    assert by_id[first[0].cylinder_id].x == pytest.approx(0.45)
