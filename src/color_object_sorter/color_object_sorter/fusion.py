"""Pure geometry and scan association for camera/LiDAR fusion."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class RangeMatch:
    """Result of associating one camera bearing with a laser scan."""

    bearing: float
    distance: float


def camera_bearing(
    normalized_x: float,
    center_normalized_x: float,
    radians_per_normalized_x: float,
) -> float:
    """Convert image x to REP-103 bearing (left is positive)."""

    return (normalized_x - center_normalized_x) * radians_per_normalized_x


def associate_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    bearing: float,
    half_window: float,
    range_min: float,
    range_max: float,
    maximum_range_jump: float,
    minimum_cluster_points: int,
) -> RangeMatch | None:
    """Return the nearest coherent range cluster around ``bearing``."""

    if angle_increment <= 0.0 or half_window <= 0.0:
        raise ValueError("scan angle increment and window must be positive")
    if maximum_range_jump < 0.0 or minimum_cluster_points < 1:
        raise ValueError("invalid cluster parameters")

    candidates: list[tuple[int, float, float]] = []
    for index, distance in enumerate(ranges):
        angle = angle_min + index * angle_increment
        error = math.atan2(math.sin(angle - bearing), math.cos(angle - bearing))
        if (
            abs(error) <= half_window
            and math.isfinite(distance)
            and range_min <= distance <= range_max
        ):
            candidates.append((index, angle, distance))

    clusters: list[list[tuple[int, float, float]]] = []
    for sample in candidates:
        if not clusters:
            clusters.append([sample])
            continue
        previous = clusters[-1][-1]
        if sample[0] == previous[0] + 1 and abs(sample[2] - previous[2]) <= maximum_range_jump:
            clusters[-1].append(sample)
        else:
            clusters.append([sample])

    coherent = [cluster for cluster in clusters if len(cluster) >= minimum_cluster_points]
    if not coherent:
        return None
    cluster = min(coherent, key=lambda item: _median([sample[2] for sample in item]))
    raw_bearing = _median([sample[1] for sample in cluster])
    return RangeMatch(
        bearing=math.atan2(math.sin(raw_bearing), math.cos(raw_bearing)),
        distance=_median([sample[2] for sample in cluster]),
    )


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0
