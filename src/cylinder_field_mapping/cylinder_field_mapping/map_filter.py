"""Occupancy-grid filtering for compact, isolated object candidates."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]

    def __post_init__(self) -> None:
        if self.width < 1 or self.height < 1 or self.resolution <= 0.0:
            raise ValueError("invalid occupancy grid geometry")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy grid data size does not match geometry")


def is_compact_map_obstacle(
    grid: GridMap,
    x: float,
    y: float,
    *,
    occupancy_threshold: int,
    search_radius: float,
    maximum_component_diameter: float,
) -> bool:
    """Reject only candidates attached to a known large occupied component.

    A live laser return may belong to a movable object that has not yet been
    incorporated into the accumulated occupancy grid. Absence of a nearby
    occupied cell is therefore not negative evidence and must pass this
    filter. The map is used only to veto returns attached to known walls or
    other large static components.
    """

    if not 0 <= occupancy_threshold <= 100:
        raise ValueError("occupancy_threshold must be in [0, 100]")
    if search_radius <= 0.0 or maximum_component_diameter <= 0.0:
        raise ValueError("map filter distances must be positive")
    center = _world_to_grid(grid, x, y)
    if center is None:
        return False
    radius_cells = max(1, math.ceil(search_radius / grid.resolution))
    seed = _nearest_occupied(grid, center, radius_cells, occupancy_threshold)
    if seed is None:
        return True

    maximum_cells = maximum_component_diameter / grid.resolution
    queue = deque([seed])
    visited = {seed}
    minimum_x = maximum_x = seed[0]
    minimum_y = maximum_y = seed[1]
    while queue:
        cell_x, cell_y = queue.popleft()
        minimum_x = min(minimum_x, cell_x)
        maximum_x = max(maximum_x, cell_x)
        minimum_y = min(minimum_y, cell_y)
        maximum_y = max(maximum_y, cell_y)
        diameter_cells = math.hypot(maximum_x - minimum_x, maximum_y - minimum_y)
        if diameter_cells * grid.resolution > maximum_component_diameter:
            return False
        for offset_x in (-1, 0, 1):
            for offset_y in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue
                neighbor = (cell_x + offset_x, cell_y + offset_y)
                if neighbor in visited or not _inside(grid, *neighbor):
                    continue
                if _value(grid, *neighbor) < occupancy_threshold:
                    continue
                visited.add(neighbor)
                queue.append(neighbor)
        if max(maximum_x - minimum_x, maximum_y - minimum_y) > maximum_cells:
            return False
    return True


def _world_to_grid(grid: GridMap, x: float, y: float) -> tuple[int, int] | None:
    dx = x - grid.origin_x
    dy = y - grid.origin_y
    cosine = math.cos(grid.origin_yaw)
    sine = math.sin(grid.origin_yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    cell = (math.floor(local_x / grid.resolution), math.floor(local_y / grid.resolution))
    return cell if _inside(grid, *cell) else None


def _nearest_occupied(
    grid: GridMap,
    center: tuple[int, int],
    radius: int,
    threshold: int,
) -> tuple[int, int] | None:
    candidates = []
    for cell_x in range(center[0] - radius, center[0] + radius + 1):
        for cell_y in range(center[1] - radius, center[1] + radius + 1):
            if not _inside(grid, cell_x, cell_y):
                continue
            distance = math.hypot(cell_x - center[0], cell_y - center[1])
            if distance <= radius and _value(grid, cell_x, cell_y) >= threshold:
                candidates.append((distance, cell_x, cell_y))
    if not candidates:
        return None
    _distance, cell_x, cell_y = min(candidates)
    return cell_x, cell_y


def _inside(grid: GridMap, x: int, y: int) -> bool:
    return 0 <= x < grid.width and 0 <= y < grid.height


def _value(grid: GridMap, x: int, y: int) -> int:
    return int(grid.data[y * grid.width + x])
