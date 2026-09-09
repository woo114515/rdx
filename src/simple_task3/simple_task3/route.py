"""Provide monotonic polyline tracking helpers with no ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


Point2D = tuple[float, float]


@dataclass(frozen=True)
class RouteProgress:
    """A lookahead target and the monotonically advancing segment cursor."""

    target: Point2D
    deviation: float
    segment_index: int


def tracking_target(
    path: Sequence[Point2D],
    position: Point2D,
    cursor: int,
    lookahead: float,
    maximum_segment_advance: int,
) -> RouteProgress:
    """Find a lookahead target without jumping to a distant overlapping leg."""

    if len(path) < 2:
        raise ValueError("path must contain at least two points")
    if lookahead <= 0.0 or not math.isfinite(lookahead):
        raise ValueError("lookahead must be positive and finite")
    if maximum_segment_advance < 1:
        raise ValueError("maximum_segment_advance must be positive")
    if not all(
        math.isfinite(value) for point in (*path, position) for value in point
    ):
        raise ValueError("path and position values must be finite")

    first = max(0, min(int(cursor), len(path) - 2))
    last = min(len(path) - 2, first + maximum_segment_advance)
    best_distance = math.inf
    best_index = first
    best_projection = path[first]
    for index in range(first, last + 1):
        start = path[index]
        end = path[index + 1]
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        squared = dx * dx + dy * dy
        ratio = 0.0
        if squared > 1e-18:
            ratio = max(
                0.0,
                min(
                    1.0,
                    ((position[0] - start[0]) * dx + (position[1] - start[1]) * dy)
                    / squared,
                ),
            )
        projection = (start[0] + ratio * dx, start[1] + ratio * dy)
        distance = math.dist(position, projection)
        if distance <= best_distance:
            best_distance = distance
            best_index = index
            best_projection = projection

    remaining = lookahead
    point = best_projection
    for endpoint in path[best_index + 1 :]:
        length = math.dist(point, endpoint)
        if length >= remaining and length > 1e-18:
            ratio = remaining / length
            target = (
                point[0] + ratio * (endpoint[0] - point[0]),
                point[1] + ratio * (endpoint[1] - point[1]),
            )
            return RouteProgress(target, best_distance, best_index)
        remaining -= length
        point = endpoint
    return RouteProgress(path[-1], best_distance, best_index)
