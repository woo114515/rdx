import math

import pytest

from cylinder_push_planner.inertial_heading import (
    InertialHeadingTracker,
    heading_is_settled,
    stable_circular_heading,
)


def test_integrates_bias_corrected_angular_rate() -> None:
    tracker = InertialHeadingTracker(maximum_gap=0.30)
    tracker.reset(0.0, 1.0, 1.1, bias=0.1)

    assert tracker.observe(1.2, 1.1, integrate=True)
    assert tracker.yaw == pytest.approx(0.2)


def test_wraps_heading_at_pi() -> None:
    tracker = InertialHeadingTracker(maximum_gap=0.30)
    tracker.reset(math.radians(179.0), 1.0, math.radians(20.0))

    assert tracker.observe(1.2, math.radians(20.0), integrate=True)
    assert math.degrees(tracker.yaw) == pytest.approx(-177.0)


def test_pause_advances_time_without_accumulating_drift() -> None:
    tracker = InertialHeadingTracker(maximum_gap=0.30)
    tracker.reset(0.4, 1.0, 0.01)

    assert tracker.observe(11.0, 0.01, integrate=False)
    assert tracker.observe(11.1, 0.01, integrate=True)
    assert tracker.yaw == pytest.approx(0.401)


def test_active_gap_is_rejected_without_integrating_it() -> None:
    tracker = InertialHeadingTracker(maximum_gap=0.30)
    tracker.reset(0.0, 1.0, 1.0)

    assert not tracker.observe(2.0, 1.0, integrate=True)
    assert tracker.yaw == pytest.approx(0.0)


def test_rejects_invalid_configuration_and_samples() -> None:
    with pytest.raises(ValueError, match="positive"):
        InertialHeadingTracker(maximum_gap=0.0)
    tracker = InertialHeadingTracker(maximum_gap=0.30)
    with pytest.raises(RuntimeError, match="not initialized"):
        tracker.observe(1.0, 0.0, integrate=True)


def test_heading_settle_requires_low_error_and_low_physical_rate() -> None:
    assert heading_is_settled(0.04, 0.02, 0.08, 0.05)
    assert not heading_is_settled(0.10, 0.02, 0.08, 0.05)
    assert not heading_is_settled(0.04, 0.20, 0.08, 0.05)


def test_stable_circular_heading_handles_wraparound() -> None:
    estimate, stable = stable_circular_heading(
        [math.radians(179.0), math.radians(-179.0), math.radians(180.0)],
        3,
        math.radians(2.0),
    )

    assert estimate is not None
    assert abs(abs(math.degrees(estimate)) - 180.0) < 0.1
    assert stable


def test_stable_circular_heading_waits_for_a_full_quiet_window() -> None:
    estimate, stable = stable_circular_heading([0.0, 0.01], 3, 0.02)
    assert estimate == pytest.approx(0.005)
    assert not stable

    estimate, stable = stable_circular_heading([0.0, 0.04, 0.0], 3, 0.02)
    assert estimate is not None
    assert not stable


def test_stable_circular_heading_rejects_invalid_limits() -> None:
    with pytest.raises(ValueError, match="limits"):
        stable_circular_heading([0.0], 0, 0.1)
