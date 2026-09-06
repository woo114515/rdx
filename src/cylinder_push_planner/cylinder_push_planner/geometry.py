"""Pure geometry for selecting the first cylinder and a staging pose."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from cylinder_field_mapping.field_map import minimum_enclosing_circle


@dataclass(frozen=True)
class Target:
    candidate_id: int
    x: float
    y: float
    color: str
    radius: float


@dataclass(frozen=True)
class SelectionPlan:
    target: Target
    envelope_x: float
    envelope_y: float
    envelope_radius: float
    staging_x: float
    staging_y: float
    staging_yaw: float


def select_right_first(
    targets: Sequence[Target],
    robot_x: float,
    robot_y: float,
    staging_clearance: float,
) -> SelectionPlan:
    """Select the rightmost target as viewed from robot toward field center."""

    if not targets:
        raise ValueError("at least one target is required")
    if staging_clearance <= 0.0:
        raise ValueError("staging_clearance must be positive")
    circle = minimum_enclosing_circle([(item.x, item.y) for item in targets])
    assert circle is not None
    cylinder_radius = max(item.radius for item in targets)
    envelope_radius = circle.radius + cylinder_radius
    forward_x = circle.x - robot_x
    forward_y = circle.y - robot_y
    norm = math.hypot(forward_x, forward_y)
    if norm < 1e-9:
        raise ValueError("robot must start outside the target field center")
    forward_x /= norm
    forward_y /= norm
    right_x, right_y = forward_y, -forward_x

    def rank(item: Target) -> tuple[float, float, int]:
        offset_x = item.x - circle.x
        offset_y = item.y - circle.y
        rightward = offset_x * right_x + offset_y * right_y
        distance = math.hypot(item.x - robot_x, item.y - robot_y)
        return rightward, -distance, -item.candidate_id

    target = max(targets, key=rank)
    radial_x = target.x - circle.x
    radial_y = target.y - circle.y
    radial_norm = math.hypot(radial_x, radial_y)
    if radial_norm < 1e-9:
        radial_x, radial_y = right_x, right_y
    else:
        radial_x /= radial_norm
        radial_y /= radial_norm
    staging_distance = envelope_radius + staging_clearance
    staging_x = circle.x + radial_x * staging_distance
    staging_y = circle.y + radial_y * staging_distance
    staging_yaw = math.atan2(target.y - staging_y, target.x - staging_x)
    return SelectionPlan(
        target=target,
        envelope_x=circle.x,
        envelope_y=circle.y,
        envelope_radius=envelope_radius,
        staging_x=staging_x,
        staging_y=staging_y,
        staging_yaw=staging_yaw,
    )
