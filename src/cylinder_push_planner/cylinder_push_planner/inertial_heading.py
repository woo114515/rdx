"""Task-local heading integration from an IMU angular-rate stream."""

from __future__ import annotations

import math
from collections.abc import Sequence


def normalize_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def heading_is_settled(
    error: float,
    angular_rate: float,
    heading_tolerance: float,
    rate_tolerance: float,
) -> bool:
    """Return whether both heading error and physical turn rate are small."""

    values = (error, angular_rate, heading_tolerance, rate_tolerance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("heading settle values must be finite")
    if heading_tolerance < 0.0 or rate_tolerance < 0.0:
        raise ValueError("heading settle tolerances must not be negative")
    return abs(error) <= heading_tolerance and abs(angular_rate) <= rate_tolerance


def stable_circular_heading(
    samples: Sequence[float], required_samples: int, maximum_spread: float
) -> tuple[float | None, bool]:
    """Estimate a wrapped heading and report whether its window is stable."""

    if required_samples < 1 or maximum_spread < 0.0:
        raise ValueError("heading window limits are invalid")
    if not samples:
        return None, False
    values = tuple(float(value) for value in samples[-required_samples:])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("heading samples must be finite")
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if math.hypot(sine, cosine) < 1e-9:
        return None, False
    estimate = math.atan2(sine, cosine)
    stable = len(values) >= required_samples and max(
        abs(normalize_angle(value - estimate)) for value in values
    ) <= maximum_spread
    return estimate, stable


class InertialHeadingTracker:
    """Integrate gyro yaw while allowing stationary collection pauses.

    The tracker is anchored once to the task-start odometry yaw. Paused samples
    advance its timestamp without changing its heading, preventing gyro bias
    from accumulating while the robot waits for a new perception snapshot.
    """

    def __init__(self, maximum_gap: float) -> None:
        if not math.isfinite(maximum_gap) or maximum_gap <= 0.0:
            raise ValueError("maximum IMU integration gap must be positive")
        self.maximum_gap = maximum_gap
        self.yaw: float | None = None
        self.last_stamp: float | None = None
        self.last_rate = 0.0
        self.bias = 0.0

    @property
    def initialized(self) -> bool:
        return self.yaw is not None

    def clear(self) -> None:
        self.yaw = None
        self.last_stamp = None
        self.last_rate = 0.0
        self.bias = 0.0

    def reset(
        self,
        reference_yaw: float,
        stamp: float,
        angular_rate: float,
        bias: float = 0.0,
    ) -> None:
        values = (reference_yaw, stamp, angular_rate, bias)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("IMU heading values must be finite")
        self.yaw = normalize_angle(reference_yaw)
        self.last_stamp = stamp
        self.last_rate = angular_rate
        self.bias = bias

    def observe(
        self,
        stamp: float,
        angular_rate: float,
        *,
        integrate: bool,
    ) -> bool:
        """Consume one sample; return false when an active gap was skipped."""

        if not self.initialized:
            raise RuntimeError("IMU heading tracker is not initialized")
        if not math.isfinite(stamp) or not math.isfinite(angular_rate):
            raise ValueError("IMU heading sample must be finite")
        assert self.last_stamp is not None
        if stamp <= self.last_stamp:
            return True
        elapsed = stamp - self.last_stamp
        previous_rate = self.last_rate
        self.last_stamp = stamp
        self.last_rate = angular_rate
        if not integrate:
            return True
        if elapsed > self.maximum_gap:
            return False
        assert self.yaw is not None
        mean_rate = 0.5 * (previous_rate + angular_rate) - self.bias
        self.yaw = normalize_angle(self.yaw + mean_rate * elapsed)
        return True
