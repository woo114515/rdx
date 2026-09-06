import pytest

from color_object_sorter.tracking import ObjectTracker
from color_object_sorter.vision import Detection


def detection(x: float, color: str = "blue") -> Detection:
    return Detection(color, 0, 0, 10, 20, 0.0, 0.0, x, 0.0, 0.01, 0.9)


def test_track_id_survives_small_motion() -> None:
    tracker = ObjectTracker(maximum_distance=0.2)
    first = tracker.update((detection(-0.5), detection(0.5)))
    second = tracker.update((detection(-0.45), detection(0.55)))
    assert [item.track_id for item in first] == [item.track_id for item in second]


def test_different_colors_do_not_share_track() -> None:
    tracker = ObjectTracker()
    first = tracker.update((detection(0.0, "blue"),))
    second = tracker.update((detection(0.0, "red"),))
    assert first[0].track_id != second[0].track_id


def test_lost_target_is_cleared_and_reassigned() -> None:
    tracker = ObjectTracker(maximum_distance=0.2, maximum_missed=1)
    first = tracker.update((detection(0.0),))
    assert len(first) == 1
    original_id = first[0].track_id

    tracker.update(())  # missed = 1
    tracker.update(())  # missed = 2 > maximum_missed, track removed

    second = tracker.update((detection(0.0),))
    assert len(second) == 1
    assert second[0].track_id != original_id


def test_invalid_tracker_limits_are_rejected() -> None:
    with pytest.raises(ValueError):
        ObjectTracker(maximum_distance=0.0)
    with pytest.raises(ValueError):
        ObjectTracker(maximum_missed=-1)
