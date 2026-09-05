import math

import pytest

from color_object_sorter.fusion import associate_scan, camera_bearing


def test_camera_center_maps_to_forward() -> None:
    assert camera_bearing(-0.069, -0.069, -math.pi / 8.0) == pytest.approx(0.0)


def test_camera_left_maps_to_positive_bearing() -> None:
    bearing = camera_bearing(-0.736, -0.069, -math.pi / 8.0)
    assert math.degrees(bearing) == pytest.approx(15.0, abs=0.1)


def test_association_selects_nearest_coherent_cluster() -> None:
    ranges = [3.0] * 21
    ranges[8:11] = [1.02, 1.0, 1.01]
    match = associate_scan(
        ranges,
        angle_min=-0.1,
        angle_increment=0.01,
        bearing=0.0,
        half_window=0.08,
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
    )
    assert match is not None
    assert match.distance == pytest.approx(1.01)


def test_association_rejects_isolated_near_noise() -> None:
    ranges = [3.0] * 21
    ranges[10] = 0.2
    match = associate_scan(
        ranges,
        angle_min=-0.1,
        angle_increment=0.01,
        bearing=0.0,
        half_window=0.08,
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
    )
    assert match is not None
    assert match.distance == pytest.approx(3.0)


def test_association_returns_none_without_cluster() -> None:
    match = associate_scan(
        [math.nan, 0.0, math.inf],
        angle_min=-0.01,
        angle_increment=0.01,
        bearing=0.0,
        half_window=0.02,
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
    )
    assert match is None


def test_association_normalizes_wrapped_scan_bearing() -> None:
    match = associate_scan(
        [1.0, 1.0, 3.0],
        angle_min=2.0 * math.pi - 0.02,
        angle_increment=0.01,
        bearing=-0.01,
        half_window=0.03,
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
    )
    assert match is not None
    assert match.bearing == pytest.approx(-0.015)
