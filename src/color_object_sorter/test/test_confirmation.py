import math

import pytest

from color_object_sorter.confirmation import Observation, TemporalConfirmer


def observation(
    stamp: float,
    color: str = "red",
    bearing: float = 0.25,
    distance: float = 1.2,
    matched: bool = True,
) -> Observation:
    return Observation(0, stamp, color, "cylinder", 0.7, bearing, distance, matched)


def test_confirms_stable_multiframe_object() -> None:
    confirmer = TemporalConfirmer(minimum_observations=3)
    result = []
    for index in range(3):
        result = confirmer.update(index * 0.1, [observation(index * 0.1)])
    assert len(result) == 1
    assert result[0].confirmed
    assert result[0].observation_count == 3
    assert result[0].detection_rate == pytest.approx(1.0)


def test_single_frame_never_confirms() -> None:
    result = TemporalConfirmer().update(0.0, [observation(0.0)])
    assert not result[0].confirmed


def test_rejects_unstable_color_vote() -> None:
    confirmer = TemporalConfirmer(minimum_observations=4)
    colors = ["red", "blue", "red", "blue"]
    for index, color in enumerate(colors):
        result = confirmer.update(index * 0.1, [observation(index * 0.1, color=color)])
    assert not result[0].confirmed
    assert result[0].color_consistency == pytest.approx(0.5)


def test_rejects_insufficient_lidar_matches() -> None:
    confirmer = TemporalConfirmer(minimum_observations=4, minimum_lidar_match_rate=0.75)
    for index in range(4):
        result = confirmer.update(
            index * 0.1,
            [observation(index * 0.1, matched=index < 2)],
        )
    assert not result[0].confirmed
    assert result[0].lidar_match_rate == pytest.approx(0.5)


def test_rejects_unstable_distance() -> None:
    confirmer = TemporalConfirmer(
        minimum_observations=4,
        association_distance=1.0,
        maximum_distance_stddev=0.1,
    )
    for index, distance in enumerate([1.0, 1.3, 1.0, 1.3]):
        result = confirmer.update(
            index * 0.1,
            [observation(index * 0.1, distance=distance)],
        )
    assert not result[0].confirmed
    assert result[0].distance_stddev > 0.1


def test_reassociates_changed_source_track_by_geometry() -> None:
    confirmer = TemporalConfirmer(minimum_observations=2)
    confirmer.update(0.0, [observation(0.0, bearing=math.radians(10.0))])
    result = confirmer.update(0.1, [observation(0.1, bearing=math.radians(11.0))])
    assert len(result) == 1
    assert result[0].confirmed


def test_missing_frames_reduce_detection_rate() -> None:
    confirmer = TemporalConfirmer(minimum_observations=2, minimum_detection_rate=0.75)
    confirmer.update(0.0, [observation(0.0)])
    confirmer.update(0.1, [])
    confirmer.update(0.2, [])
    result = confirmer.update(0.3, [observation(0.3)])
    assert not result[0].confirmed
    assert result[0].detection_rate == pytest.approx(0.5)


def test_missing_current_frame_reports_occluded() -> None:
    confirmer = TemporalConfirmer(minimum_observations=2)
    confirmer.update(0.0, [observation(0.0)])
    confirmer.update(0.1, [observation(0.1)])
    result = confirmer.update(0.2, [])
    assert result[0].state == "occluded"
    assert not result[0].confirmed


def test_ambiguous_current_observation_cannot_confirm() -> None:
    confirmer = TemporalConfirmer(minimum_observations=2)
    confirmer.update(0.0, [observation(0.0)])
    item = observation(0.1)
    ambiguous = Observation(
        item.frame,
        item.stamp,
        item.color,
        item.shape,
        item.confidence,
        item.bearing,
        math.nan,
        False,
        "ambiguous",
    )
    result = confirmer.update(0.1, [ambiguous])
    assert result[0].state == "ambiguous"
    assert not result[0].confirmed
