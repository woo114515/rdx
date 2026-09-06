"""Pure geometry and one-to-one camera/LiDAR association."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from typing import Sequence


@dataclass(frozen=True)
class LaserCluster:
    """A coherent contiguous group of laser returns."""

    bearing: float
    distance: float
    point_count: int


@dataclass(frozen=True)
class RangeMatch:
    """Result of associating one camera bearing with a laser cluster."""

    bearing: float
    distance: float


@dataclass(frozen=True)
class Association:
    """One detection's globally allocated cluster and safety status."""

    cluster_index: int | None
    status: str


def camera_bearing(
    normalized_x: float,
    center_normalized_x: float,
    radians_per_normalized_x: float,
) -> float:
    """Convert image x to REP-103 bearing (left is positive)."""

    return (normalized_x - center_normalized_x) * radians_per_normalized_x


def extract_scan_clusters(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    maximum_range_jump: float,
    minimum_cluster_points: int,
) -> tuple[LaserCluster, ...]:
    """Extract range-contiguous clusters from a circular laser scan."""

    if angle_increment <= 0.0:
        raise ValueError("scan angle increment must be positive")
    if maximum_range_jump < 0.0 or minimum_cluster_points < 1:
        raise ValueError("invalid cluster parameters")

    groups: list[list[tuple[int, float, float]]] = []
    for index, distance in enumerate(ranges):
        valid = math.isfinite(distance) and range_min <= distance <= range_max
        if not valid:
            continue
        sample = (index, angle_min + index * angle_increment, distance)
        if not groups:
            groups.append([sample])
            continue
        previous = groups[-1][-1]
        if (
            index == previous[0] + 1
            and abs(distance - previous[2]) <= maximum_range_jump
        ):
            groups[-1].append(sample)
        else:
            groups.append([sample])

    if (
        len(groups) > 1
        and len(ranges) * angle_increment >= 2.0 * math.pi - 2.0 * angle_increment
        and groups[0][0][0] == 0
        and groups[-1][-1][0] == len(ranges) - 1
        and abs(groups[0][0][2] - groups[-1][-1][2]) <= maximum_range_jump
    ):
        groups[0] = groups[-1] + groups[0]
        groups.pop()

    result = []
    for group in groups:
        if len(group) < minimum_cluster_points:
            continue
        result.append(
            LaserCluster(
                bearing=_circular_mean([sample[1] for sample in group]),
                distance=_median([sample[2] for sample in group]),
                point_count=len(group),
            )
        )
    return tuple(result)


def associate_one_to_one(
    predicted_bearings: Sequence[float],
    half_windows: Sequence[float],
    clusters: Sequence[LaserCluster],
    ambiguity_margin: float,
) -> tuple[Association, ...]:
    """Globally allocate clusters, rejecting close alternatives as ambiguous."""

    if len(predicted_bearings) != len(half_windows):
        raise ValueError("each detection needs an association window")
    if ambiguity_margin < 0.0 or any(window <= 0.0 for window in half_windows):
        raise ValueError("invalid association limits")

    candidates: list[tuple[tuple[int, float], ...]] = []
    for bearing, window in zip(predicted_bearings, half_windows):
        items = []
        for index, cluster in enumerate(clusters):
            error = abs(_angle_difference(cluster.bearing, bearing))
            if error <= window:
                items.append((index, error))
        candidates.append(tuple(sorted(items, key=lambda item: (item[1], item[0]))))

    @lru_cache(maxsize=None)
    def solve(
        position: int,
        used: tuple[int, ...],
    ) -> tuple[int, float, tuple[int | None, ...]]:
        if position == len(candidates):
            return 0, 0.0, ()
        used_set = set(used)
        options: list[tuple[int, float, tuple[int | None, ...]]] = []
        count, cost, tail = solve(position + 1, used)
        options.append((count, cost, (None,) + tail))
        for cluster_index, error in candidates[position]:
            if cluster_index in used_set:
                continue
            next_used = tuple(sorted((*used, cluster_index)))
            count, cost, tail = solve(position + 1, next_used)
            options.append((count + 1, cost + error, (cluster_index,) + tail))
        return min(
            options,
            key=lambda item: (-item[0], item[1], _assignment_key(item[2])),
        )

    _, _, assignment = solve(0, ())
    result = []
    for detection_index, cluster_index in enumerate(assignment):
        own_candidates = dict(candidates[detection_index])
        if cluster_index is None:
            status = "ambiguous" if own_candidates else "unmatched"
            result.append(Association(None, status))
            continue
        assigned_error = own_candidates[cluster_index]
        ambiguous = any(
            other_index != cluster_index
            and abs(other_error - assigned_error) <= ambiguity_margin
            for other_index, other_error in candidates[detection_index]
        )
        for other_detection, other_candidates in enumerate(candidates):
            if other_detection == detection_index:
                continue
            other_error = dict(other_candidates).get(cluster_index)
            if (
                other_error is not None
                and abs(other_error - assigned_error) <= ambiguity_margin
            ):
                ambiguous = True
                break
        result.append(
            Association(
                None if ambiguous else cluster_index,
                "ambiguous" if ambiguous else "matched",
            )
        )
    return tuple(result)


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
    """Backward-compatible single-detection association helper."""

    clusters = extract_scan_clusters(
        ranges,
        angle_min,
        angle_increment,
        range_min,
        range_max,
        maximum_range_jump,
        minimum_cluster_points,
    )
    associations = associate_one_to_one(
        (bearing,), (half_window,), clusters, ambiguity_margin=0.0
    )
    index = associations[0].cluster_index
    if index is None:
        return None
    cluster = clusters[index]
    return RangeMatch(cluster.bearing, cluster.distance)


def _assignment_key(assignment: tuple[int | None, ...]) -> tuple[int, ...]:
    return tuple(10**9 if item is None else item for item in assignment)


def _angle_difference(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


def _circular_mean(values: Sequence[float]) -> float:
    return math.atan2(
        sum(math.sin(value) for value in values),
        sum(math.cos(value) for value in values),
    )


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) / 2.0
