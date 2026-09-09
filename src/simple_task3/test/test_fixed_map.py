"""Tests for fixed-map persistence and handoff geometry."""

import math
from types import SimpleNamespace

import pytest

from simple_task3.fixed_map import map_yaml, occupancy_to_pgm, pose_distance


def test_occupancy_grid_is_flipped_and_encoded_for_map_server() -> None:
    encoded = occupancy_to_pgm([0, 100, -1, 50], 2, 2)
    assert encoded.endswith(bytes([205, 127, 254, 0]))


def test_occupancy_grid_rejects_bad_dimensions() -> None:
    with pytest.raises(ValueError):
        occupancy_to_pgm([0], 2, 2)


def test_map_yaml_preserves_origin_yaw() -> None:
    origin = SimpleNamespace(
        position=SimpleNamespace(x=-1.0, y=2.0),
        orientation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sin(0.25),
            w=math.cos(0.25),
        ),
    )
    document = map_yaml("task3_initial.pgm", 0.05, origin)
    assert "image: task3_initial.pgm" in document
    assert "resolution: 0.05" in document
    assert "origin: [-1, 2, 0.5]" in document


def test_handoff_pose_error_wraps_yaw() -> None:
    position, yaw = pose_distance(
        (0.0, 0.0, math.pi - 0.02),
        (0.03, 0.04, -math.pi + 0.01),
    )
    assert position == pytest.approx(0.05)
    assert yaw == pytest.approx(0.03)
