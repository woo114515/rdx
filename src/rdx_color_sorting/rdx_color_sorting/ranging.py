"""Pure helpers for associating camera detections with laser ranges."""

from __future__ import annotations

import math
from statistics import median
from typing import Sequence


def range_near_bearing(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    bearing: float,
    half_angle: float,
    minimum_samples: int,
) -> float | None:
    """Return the median valid laser range around a camera bearing."""
    if angle_increment == 0.0:
        raise ValueError("angle_increment must not be zero")
    if half_angle <= 0.0:
        raise ValueError("half_angle must be positive")
    if minimum_samples <= 0:
        raise ValueError("minimum_samples must be positive")

    samples: list[float] = []
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance):
            continue
        if distance < max(0.0, range_min) or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        angle_error = math.atan2(
            math.sin(angle - bearing),
            math.cos(angle - bearing),
        )
        if abs(angle_error) <= half_angle:
            samples.append(float(distance))
    if len(samples) < minimum_samples:
        return None
    return float(median(samples))
