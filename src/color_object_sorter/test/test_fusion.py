import math

import pytest

from color_object_sorter.fusion import (
    LaserCluster,
    associate_one_to_one,
    associate_scan,
    camera_bearing,
    extract_scan_clusters,
)


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


def test_extracts_distinct_range_clusters() -> None:
    clusters = extract_scan_clusters(
        [3.0, 3.0, math.nan, 1.0, 1.02, math.nan, 2.0, 2.01],
        angle_min=-0.04,
        angle_increment=0.01,
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
    )
    assert [item.distance for item in clusters] == pytest.approx([3.0, 1.01, 2.005])


def test_global_assignment_never_reuses_cluster() -> None:
    clusters = (
        LaserCluster(math.radians(0.0), 1.0, 3),
        LaserCluster(math.radians(4.0), 1.2, 3),
    )
    result = associate_one_to_one(
        [math.radians(0.5), math.radians(3.0)],
        [math.radians(5.0), math.radians(5.0)],
        clusters,
        ambiguity_margin=math.radians(0.2),
    )
    indices = [item.cluster_index for item in result if item.cluster_index is not None]
    assert len(indices) == len(set(indices)) == 2


def test_competing_visual_detections_are_ambiguous() -> None:
    clusters = (LaserCluster(0.0, 1.0, 3),)
    result = associate_one_to_one(
        [math.radians(-0.2), math.radians(0.2)],
        [math.radians(4.0), math.radians(4.0)],
        clusters,
        ambiguity_margin=math.radians(1.0),
    )
    assert [item.status for item in result] == ["ambiguous", "ambiguous"]
    assert all(item.cluster_index is None for item in result)


def test_merged_wide_detection_rejects_two_lidar_clusters() -> None:
    clusters = (
        LaserCluster(math.radians(-2.0), 0.8, 3),
        LaserCluster(math.radians(2.0), 1.4, 3),
    )
    result = associate_one_to_one(
        [0.0],
        [math.radians(4.0)],
        clusters,
        ambiguity_margin=math.radians(0.5),
    )
    assert result[0].status == "ambiguous"
    assert result[0].cluster_index is None


def test_far_background_is_removed_before_ambiguity_check() -> None:
    clusters = (
        LaserCluster(math.radians(1.4), 1.50, 4),
        LaserCluster(math.radians(0.6), 4.55, 2),
    )
    result = associate_one_to_one(
        [0.0],
        [math.radians(4.0)],
        clusters,
        ambiguity_margin=math.radians(1.0),
        maximum_depth_gap=0.35,
    )
    assert result[0].status == "matched"
    assert result[0].cluster_index == 0


def test_oversized_contiguous_return_is_split() -> None:
    clusters = extract_scan_clusters(
        [1.5] * 10,
        angle_min=0.0,
        angle_increment=math.radians(1.0),
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
        maximum_angular_span=math.radians(4.0),
    )
    assert len(clusters) == 3
    assert [item.point_count for item in clusters] == [4, 4, 2]


def test_two_detections_can_use_two_halves_of_a_wide_return() -> None:
    clusters = extract_scan_clusters(
        [1.5] * 8,
        angle_min=math.radians(1.0),
        angle_increment=math.radians(1.0),
        range_min=0.15,
        range_max=20.0,
        maximum_range_jump=0.15,
        minimum_cluster_points=2,
        maximum_angular_span=math.radians(4.0),
    )
    result = associate_one_to_one(
        [math.radians(3.0), math.radians(7.0)],
        [math.radians(3.0), math.radians(3.0)],
        clusters,
        ambiguity_margin=math.radians(0.5),
        maximum_depth_gap=0.35,
    )
    assert [item.status for item in result] == ["matched", "matched"]
    assert result[0].cluster_index != result[1].cluster_index
