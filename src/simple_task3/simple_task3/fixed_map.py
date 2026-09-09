"""Pure helpers for the Task 3 fixed-map localization handoff."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence


def occupancy_to_pgm(data: Sequence[int], width: int, height: int) -> bytes:
    """Encode ROS OccupancyGrid cells as a map_server-compatible PGM."""

    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError("occupancy grid dimensions do not match its data")
    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        for column in range(width):
            value = int(data[row * width + column])
            if value < 0:
                pixels.append(205)
            elif value >= 65:
                pixels.append(0)
            elif value <= 25:
                pixels.append(254)
            else:
                pixels.append(round(254.0 - value * 254.0 / 100.0))
    header = f"P5\n# CREATOR: simple_task3 fixed map\n{width} {height}\n255\n"
    return header.encode() + bytes(pixels)


def map_yaml(image_name: str, resolution: float, origin) -> str:
    """Return a map_server YAML document without requiring PyYAML at runtime."""

    if not image_name or not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("map image name and resolution must be valid")
    yaw = math.atan2(
        2.0
        * (
            origin.orientation.w * origin.orientation.z
            + origin.orientation.x * origin.orientation.y
        ),
        1.0 - 2.0 * (origin.orientation.y**2 + origin.orientation.z**2),
    )
    return (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {resolution:.12g}\n"
        f"origin: [{origin.position.x:.12g}, {origin.position.y:.12g}, {yaw:.12g}]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )


def atomic_write_map(directory: Path, basename: str, grid) -> tuple[Path, Path]:
    """Persist a captured OccupancyGrid using atomic file replacements."""

    if not basename or Path(basename).name != basename:
        raise ValueError("map basename must be one path component")
    directory.mkdir(parents=True, exist_ok=True)
    pgm_path = directory / f"{basename}.pgm"
    yaml_path = directory / f"{basename}.yaml"
    pgm_temporary = directory / f".{basename}.pgm.tmp"
    yaml_temporary = directory / f".{basename}.yaml.tmp"
    pgm_temporary.write_bytes(
        occupancy_to_pgm(grid.data, grid.info.width, grid.info.height)
    )
    yaml_temporary.write_text(
        map_yaml(pgm_path.name, grid.info.resolution, grid.info.origin),
        encoding="utf-8",
    )
    pgm_temporary.replace(pgm_path)
    yaml_temporary.replace(yaml_path)
    return yaml_path, pgm_path


def pose_distance(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float]:
    """Return planar and wrapped-yaw separation for handoff verification."""

    position = math.dist(first[:2], second[:2])
    delta = second[2] - first[2]
    return position, abs(math.atan2(math.sin(delta), math.cos(delta)))
