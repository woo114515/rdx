import math

import pytest

from cylinder_field_mapping.inventory import ObjectInventory
from cylinder_field_mapping.lidar_candidates import (
    CandidateAccumulator,
    extract_candidates,
    merge_nearby_observations,
    select_primary_spatial_group,
)
from cylinder_field_mapping.map_filter import (
    GridMap,
    inside_exclusion_zone,
    is_compact_map_obstacle,
)


def test_extracts_compact_cluster_and_rejects_long_surface() -> None:
    ranges = [math.inf] * 80
    ranges[10:14] = [1.00, 0.99, 0.99, 1.00]
    ranges[30:55] = [2.0] * 25
    result = extract_candidates(
        ranges,
        angle_min=-0.4,
        angle_increment=0.01,
        range_min=0.15,
        range_max=10.0,
        maximum_point_gap=0.08,
        minimum_points=2,
        minimum_diameter=0.015,
        maximum_diameter=0.16,
        nominal_radius=0.035,
    )
    assert len(result) == 1
    assert result[0].point_count == 4
    assert math.hypot(result[0].x, result[0].y) == pytest.approx(1.025, abs=0.02)


def test_invalid_ranges_split_clusters() -> None:
    ranges = [1.0, 1.0, math.inf, 1.0, 1.0]
    result = extract_candidates(
        ranges,
        angle_min=-0.02,
        angle_increment=0.01,
        range_min=0.15,
        range_max=10.0,
        minimum_diameter=0.005,
        maximum_diameter=0.10,
    )
    assert len(result) == 2


def test_full_circle_scan_merges_cluster_across_zero_angle() -> None:
    sample_count = 360
    ranges = [math.inf] * sample_count
    ranges[0] = 2.11
    ranges[-1] = 2.11

    result = extract_candidates(
        ranges,
        angle_min=0.0,
        angle_increment=2.0 * math.pi / sample_count,
        range_min=0.15,
        range_max=10.0,
    )

    assert len(result) == 1
    assert result[0].point_count == 2
    assert math.hypot(result[0].x, result[0].y) == pytest.approx(2.145)


def test_limited_scan_does_not_merge_its_array_ends() -> None:
    ranges = [math.inf] * 80
    ranges[0] = 2.11
    ranges[-1] = 2.11

    result = extract_candidates(
        ranges,
        angle_min=-0.4,
        angle_increment=0.01,
        range_min=0.15,
        range_max=10.0,
    )

    assert result == ()


def test_accumulator_keeps_candidates_one_to_one() -> None:
    accumulator = CandidateAccumulator(association_distance=0.12)
    accumulator.update(((0.0, 0.0, 0.035, 0.8),), 1.0)
    result = accumulator.update(
        (
            (0.01, 0.0, 0.035, 0.9),
            (0.08, 0.0, 0.035, 0.7),
        ),
        2.0,
    )
    assert len(result) == 2
    assert sorted(item.observation_count for item in result) == [1, 2]


def test_stable_candidates_obey_observation_threshold() -> None:
    accumulator = CandidateAccumulator()
    accumulator.update(((1.0, 2.0, 0.035, 0.8),), 1.0)
    assert accumulator.stable(2) == ()
    accumulator.update(((1.01, 2.0, 0.035, 0.9),), 2.0)
    assert len(accumulator.stable(2)) == 1


def test_accumulator_removes_track_after_consecutive_misses() -> None:
    accumulator = CandidateAccumulator(maximum_missed_updates=2)
    accumulator.update(((1.0, 0.0, 0.035, 0.8),), 1.0)
    accumulator.update((), 2.0)
    assert len(accumulator.update((), 3.0)) == 1
    assert accumulator.update((), 4.0) == ()


def test_accumulator_keeps_id_across_brief_dropout() -> None:
    accumulator = CandidateAccumulator(maximum_missed_updates=2)
    first = accumulator.update(((1.0, 0.0, 0.035, 0.8),), 1.0)[0]
    accumulator.update((), 2.0)
    resumed = accumulator.update(((1.01, 0.0, 0.035, 0.9),), 3.0)[0]
    assert resumed.candidate_id == first.candidate_id
    assert resumed.observation_count == 2


def test_merges_fragments_below_configured_object_separation() -> None:
    result = merge_nearby_observations(
        (
            (1.0, 1.0, 0.035, 0.6),
            (1.08, 1.02, 0.035, 0.9),
            (1.35, 1.0, 0.035, 0.8),
        ),
        minimum_separation=0.20,
    )
    assert len(result) == 2
    assert max(item[3] for item in result) == pytest.approx(0.9)


def test_map_filter_accepts_compact_component_and_rejects_wall() -> None:
    data = [0] * 100
    for x in range(1, 9):
        data[2 * 10 + x] = 100
    data[7 * 10 + 7] = 100
    data[7 * 10 + 8] = 100
    grid = GridMap(10, 10, 0.05, 0.0, 0.0, 0.0, data)
    options = {
        "occupancy_threshold": 50,
        "search_radius": 0.10,
        "maximum_component_diameter": 0.20,
    }
    assert not is_compact_map_obstacle(grid, 0.25, 0.125, **options)
    assert is_compact_map_obstacle(grid, 0.375, 0.375, **options)


def test_map_filter_accepts_live_candidate_absent_from_stale_map() -> None:
    grid = GridMap(5, 5, 0.05, 0.0, 0.0, 0.0, [0] * 25)
    assert is_compact_map_obstacle(
        grid,
        0.1,
        0.1,
        occupancy_threshold=50,
        search_radius=0.05,
        maximum_component_diameter=0.20,
    )


def test_delivered_object_exclusion_is_bounded() -> None:
    centers = ((1.0, 2.0), (3.0, 4.0))

    assert inside_exclusion_zone(1.10, 2.0, centers, 0.15)
    assert not inside_exclusion_zone(1.20, 2.0, centers, 0.15)
    assert not inside_exclusion_zone(1.0, 2.0, centers, 0.0)
    with pytest.raises(ValueError):
        inside_exclusion_zone(1.0, 2.0, centers, -0.1)


def test_inventory_is_extensible_and_computes_total() -> None:
    inventory = ObjectInventory.from_lists(
        ["blue", "green", "red", "yellow"], [2, 1, 3, 4]
    )
    assert inventory.total == 10
    assert inventory.colors[-1] == "yellow"
    assert inventory.color_counts == (2, 1, 3, 4)


def test_inventory_allows_exhausted_color_for_multi_push_cycles() -> None:
    inventory = ObjectInventory.from_lists(["blue", "green", "red"], [0, 2, 2])

    assert inventory.total == 4
    assert inventory.color_counts == (0, 2, 2)


def test_selects_largest_connected_candidate_group() -> None:
    accumulator = CandidateAccumulator(association_distance=0.1)
    candidates = accumulator.update(
        (
            (0.0, 0.0, 0.035, 0.8),
            (0.3, 0.0, 0.035, 0.8),
            (0.6, 0.0, 0.035, 0.8),
            (3.0, 2.0, 0.035, 0.9),
            (3.2, 2.0, 0.035, 0.9),
        ),
        1.0,
    )
    selected = select_primary_spatial_group(candidates, 0.45)
    assert [item.candidate_id for item in selected] == [1, 2, 3]


@pytest.mark.parametrize(
    ("colors", "counts"),
    [
        (["blue"], []),
        (["blue", "blue"], [1, 1]),
        (["blue"], [-1]),
        (["blue", "green"], [0, 0]),
    ],
)
def test_inventory_rejects_invalid_configuration(colors, counts) -> None:
    with pytest.raises(ValueError):
        ObjectInventory.from_lists(colors, counts)
