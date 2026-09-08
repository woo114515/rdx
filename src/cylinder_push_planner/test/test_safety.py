import pytest

from cylinder_push_planner.safety import navigation_safety_fault


def test_missing_scan_requires_stop() -> None:
    assert "missing" in navigation_safety_fault(10.0, 0.0, 9.0, 1.0, 20.0)


def test_stale_scan_requires_stop() -> None:
    assert "stale" in navigation_safety_fault(10.0, 8.5, 8.0, 1.0, 20.0)


def test_navigation_timeout_requires_stop() -> None:
    assert "timed out" in navigation_safety_fault(30.0, 29.9, 5.0, 1.0, 20.0)


def test_fresh_scan_and_bounded_navigation_are_safe() -> None:
    assert navigation_safety_fault(10.0, 9.5, 5.0, 1.0, 20.0) is None


def test_invalid_watchdog_configuration_is_rejected() -> None:
    with pytest.raises(ValueError):
        navigation_safety_fault(10.0, 9.5, 5.0, 0.0, 20.0)
