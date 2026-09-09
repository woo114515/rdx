"""Test route-tracking helpers."""

import pytest

from simple_task3.route import tracking_target


def test_cursor_never_moves_backwards() -> None:
    path = ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0))
    result = tracking_target(path, (1.7, 0.1), 1, 0.3, 2)
    assert result.segment_index >= 1
    assert result.deviation == pytest.approx(0.1)


def test_overlapping_distant_leg_is_not_selected() -> None:
    path = (
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (1.0, 0.0),
        (0.0, 0.0),
    )
    result = tracking_target(path, (0.1, 0.0), 0, 0.2, 1)
    assert result.segment_index == 0
    assert result.target == pytest.approx((0.3, 0.0))


def test_progress_can_advance_by_bounded_window() -> None:
    path = tuple((float(index), 0.0) for index in range(8))
    result = tracking_target(path, (5.2, 0.0), 1, 0.2, 2)
    assert result.segment_index == 3


@pytest.mark.parametrize("lookahead", [0.0, -0.1, float("nan")])
def test_invalid_lookahead_is_rejected(lookahead: float) -> None:
    with pytest.raises(ValueError):
        tracking_target(((0.0, 0.0), (1.0, 0.0)), (0.0, 0.0), 0, lookahead, 2)
