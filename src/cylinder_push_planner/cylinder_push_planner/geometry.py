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


@dataclass(frozen=True)
class Keepout:
    """Convex hull of the unselected cylinders and its expanded boundary."""

    hull: tuple[tuple[float, float], ...]
    boundary: tuple[tuple[float, float], ...]
    clearance: float


@dataclass(frozen=True)
class PushPreviewPlan:
    """Motion-free geometry for one complete push-and-return preview."""

    selection: SelectionPlan
    keepout: Keepout
    home_x: float
    home_y: float
    home_yaw: float
    destination_x: float
    destination_y: float
    staging_x: float
    staging_y: float
    staging_yaw: float
    approach_path: tuple[tuple[float, float], ...]
    target_path: tuple[tuple[float, float], ...]
    robot_push_path: tuple[tuple[float, float], ...]
    release_x: float
    release_y: float
    return_path: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class Viewpoint:
    """One outside-envelope pose used to remove LiDAR occlusion."""

    side: str
    x: float
    y: float
    yaw: float
    envelope_x: float
    envelope_y: float
    envelope_radius: float
    minimum_center_radius: float


def generate_reobservation_viewpoints(
    targets: Sequence[Target],
    robot_x: float,
    robot_y: float,
    group_clearance: float,
    travel_distance: float,
    minimum_angle: float,
    maximum_angle: float,
) -> tuple[Viewpoint, Viewpoint]:
    """Return right and left viewpoints on a safe circle around the group."""

    if not targets:
        raise ValueError("at least one target is required")
    if group_clearance <= 0.0:
        raise ValueError("group_clearance must be positive")
    if travel_distance <= 0.0:
        raise ValueError("travel_distance must be positive")
    if not 0.0 < minimum_angle <= maximum_angle < math.pi:
        raise ValueError("viewpoint angle limits are invalid")

    circle = minimum_enclosing_circle([(item.x, item.y) for item in targets])
    assert circle is not None
    object_radius = max(item.radius for item in targets)
    envelope_radius = circle.radius + object_radius
    robot_dx = robot_x - circle.x
    robot_dy = robot_y - circle.y
    robot_radius = math.hypot(robot_dx, robot_dy)
    if robot_radius < 1e-9:
        raise ValueError("robot must start outside the target field center")

    safe_radius = max(robot_radius, envelope_radius + group_clearance)
    offset = max(minimum_angle, min(maximum_angle, travel_distance / safe_radius))
    start_angle = math.atan2(robot_dy, robot_dx)

    def make(side: str, signed_offset: float) -> Viewpoint:
        angle = start_angle + signed_offset
        x = circle.x + safe_radius * math.cos(angle)
        y = circle.y + safe_radius * math.sin(angle)
        return Viewpoint(
            side=side,
            x=x,
            y=y,
            yaw=math.atan2(circle.y - y, circle.x - x),
            envelope_x=circle.x,
            envelope_y=circle.y,
            envelope_radius=envelope_radius,
            minimum_center_radius=envelope_radius + group_clearance,
        )

    # When the robot faces the group center, increasing the polar angle moves
    # toward its local right side.
    return make("right", offset), make("left", -offset)


def path_length(points: Sequence[tuple[float, float]]) -> float:
    """Return planar polyline length."""

    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(points, points[1:])
    )


def destination_slot(
    center_x: float,
    center_y: float,
    slot_index: int,
    slot_count: int,
    spacing: float,
) -> tuple[float, float]:
    """Spread same-color destinations tangentially around a local-frame center."""

    if slot_count < 1:
        raise ValueError("destination slot count must be positive")
    if not 0 <= slot_index < slot_count:
        raise ValueError("destination slot index is outside its configured count")
    if spacing <= 0.0:
        raise ValueError("destination slot spacing must be positive")
    radius = math.hypot(center_x, center_y)
    if radius <= 1e-9:
        raise ValueError("destination center must differ from task-start pose")
    offset = (slot_index - 0.5 * (slot_count - 1)) * spacing
    tangent_x = -center_y / radius
    tangent_y = center_x / radius
    return center_x + tangent_x * offset, center_y + tangent_y * offset


def path_stays_outside(
    points: Sequence[tuple[float, float]],
    center_x: float,
    center_y: float,
    minimum_radius: float,
) -> bool:
    """Return whether every sampled path pose stays outside the group."""

    if minimum_radius <= 0.0:
        raise ValueError("minimum_radius must be positive")
    return bool(points) and all(
        math.hypot(x - center_x, y - center_y) >= minimum_radius
        for x, y in points
    )


def point_sets_stable(
    previous: Sequence[tuple[float, float]],
    current: Sequence[tuple[float, float]],
    maximum_motion: float,
) -> bool:
    """Return whether two unordered snapshots describe the same stable points."""

    if maximum_motion <= 0.0:
        raise ValueError("maximum_motion must be positive")
    if len(previous) != len(current) or not current:
        return False
    unmatched = list(previous)
    for point in current:
        nearest = min(
            range(len(unmatched)),
            key=lambda index: math.dist(point, unmatched[index]),
        )
        if math.dist(point, unmatched[nearest]) > maximum_motion:
            return False
        unmatched.pop(nearest)
    return True


def points_near_reference(
    points: Sequence[tuple[float, float]],
    reference: Sequence[tuple[float, float]],
    maximum_distance: float,
) -> bool:
    """Reject candidates that cannot belong to a previously accepted field."""

    if maximum_distance <= 0.0:
        raise ValueError("maximum_distance must be positive")
    if not points or not reference:
        return False
    return all(
        min(math.dist(point, anchor) for anchor in reference) <= maximum_distance
        for point in points
    )


def convex_hull(
    points: Sequence[tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
    """Return the counter-clockwise convex hull without repeating its start."""

    unique = sorted(set(points))
    if len(unique) <= 1:
        return tuple(unique)

    def cross(origin, first, second) -> float:
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def expand_convex_hull(
    hull: Sequence[tuple[float, float]],
    clearance: float,
    arc_steps: int = 32,
) -> tuple[tuple[float, float], ...]:
    """Approximate the hull Minkowski sum with a clearance-radius disk."""

    if clearance <= 0.0:
        raise ValueError("keepout clearance must be positive")
    if arc_steps < 2:
        raise ValueError("arc_steps must be at least two")
    if not hull:
        return ()
    samples = tuple(
        (
            vertex[0] + clearance * math.cos(2.0 * math.pi * index / arc_steps),
            vertex[1] + clearance * math.sin(2.0 * math.pi * index / arc_steps),
        )
        for vertex in hull
        for index in range(arc_steps)
    )
    return convex_hull(samples)


def build_remaining_keepout(
    targets: Sequence[Target],
    excluded_candidate_id: int,
    clearance: float,
) -> Keepout:
    """Build a keepout from all cylinders except the selected target."""

    remaining = tuple(
        (item.x, item.y)
        for item in targets
        if item.candidate_id != excluded_candidate_id
    )
    if len(remaining) != len(targets) - 1:
        raise ValueError("selected candidate ID must occur exactly once")
    hull = convex_hull(remaining)
    return Keepout(hull, expand_convex_hull(hull, clearance), clearance)


def point_outside_keepout(
    point: tuple[float, float],
    keepout: Keepout,
    tolerance: float = 1e-6,
) -> bool:
    """Return whether a point lies outside a convex hull plus clearance."""

    if not keepout.hull:
        return True
    return _distance_to_hull(point, keepout.hull) + tolerance >= keepout.clearance


def polyline_outside_keepout(
    points: Sequence[tuple[float, float]],
    keepout: Keepout,
    sample_spacing: float = 0.02,
) -> bool:
    """Check all segments of a polyline against an expanded hull."""

    if sample_spacing <= 0.0:
        raise ValueError("sample spacing must be positive")
    if not points:
        return False
    for first, second in zip(points, points[1:]):
        length = math.dist(first, second)
        steps = max(1, math.ceil(length / sample_spacing))
        for index in range(steps + 1):
            ratio = index / steps
            point = (
                first[0] + ratio * (second[0] - first[0]),
                first[1] + ratio * (second[1] - first[1]),
            )
            if not point_outside_keepout(point, keepout):
                return False
    return point_outside_keepout(points[-1], keepout)


def build_push_preview(
    targets: Sequence[Target],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    destination_x: float,
    destination_y: float,
    staging_clearance: float,
    robot_clearance: float,
    contact_offset: float,
    release_retreat: float,
    envelope_exit_clearance: float = 0.10,
    selection: SelectionPlan | None = None,
    delivered_exclusion_radius: float = 0.0,
    task_home: tuple[float, float, float] | None = None,
) -> PushPreviewPlan:
    """Build a collision-screened geometric preview without commanding motion."""

    if contact_offset <= 0.0 or release_retreat <= 0.0:
        raise ValueError("contact offset and release retreat must be positive")
    if envelope_exit_clearance < 0.0:
        raise ValueError("envelope exit clearance must not be negative")
    if delivered_exclusion_radius < 0.0:
        raise ValueError("delivered exclusion radius must not be negative")
    if task_home is None:
        home_x, home_y, home_yaw = robot_x, robot_y, robot_yaw
    else:
        home_x, home_y, home_yaw = (float(value) for value in task_home)
        if not all(math.isfinite(value) for value in (home_x, home_y, home_yaw)):
            raise ValueError("task home pose must be finite")
    if selection is None:
        selection = select_right_first(
            targets, robot_x, robot_y, staging_clearance
        )
    elif selection.target not in targets:
        raise ValueError("selection target must belong to targets")
    keepout = build_remaining_keepout(
        targets, selection.target.candidate_id, robot_clearance
    )
    destination = (destination_x, destination_y)
    target_start = (selection.target.x, selection.target.y)
    if math.dist(target_start, destination) < 0.25:
        raise ValueError("destination must be separated from the selected target")
    destination_from_initial_center = math.hypot(
        destination_x - selection.envelope_x,
        destination_y - selection.envelope_y,
    )
    if (
        destination_from_initial_center
        < selection.envelope_radius + envelope_exit_clearance
    ):
        raise ValueError("destination must be outside the initial cylinder envelope")
    remaining_targets = tuple(
        item
        for item in targets
        if item.candidate_id != selection.target.candidate_id
    )
    if any(
        math.dist(destination, (item.x, item.y))
        <= delivered_exclusion_radius + item.radius
        for item in remaining_targets
    ):
        raise ValueError(
            "destination exclusion zone overlaps a remaining cylinder"
        )

    candidates = _curve_candidates(target_start, destination, keepout)
    best = None
    for target_path in candidates:
        robot_path = _offset_behind_path(target_path, contact_offset)
        if not polyline_outside_keepout(robot_path, keepout):
            continue
        target_keepout = Keepout(
            keepout.hull,
            keepout.boundary,
            max(item.radius for item in targets) + 0.03,
        )
        if not polyline_outside_keepout(target_path, target_keepout):
            continue
        first_heading = math.atan2(
            target_path[1][1] - target_path[0][1],
            target_path[1][0] - target_path[0][0],
        )
        staging = (
            robot_path[0][0] - staging_clearance * math.cos(first_heading),
            robot_path[0][1] - staging_clearance * math.sin(first_heading),
        )
        if not polyline_outside_keepout((staging, robot_path[0]), keepout):
            continue
        score = path_length(robot_path)
        if best is None or score < best[0]:
            best = score, target_path, robot_path, staging, first_heading
    if best is None:
        raise ValueError("no push curve stays outside the remaining-cylinder keepout")

    _, target_path, robot_path, staging, first_heading = best
    # Nav2 must stop before the target enters the robot footprint/inflation
    # zone. A later direct, monitored contact stage covers this short segment.
    staging_x, staging_y = staging
    last_heading = math.atan2(
        target_path[-1][1] - target_path[-2][1],
        target_path[-1][0] - target_path[-2][0],
    )
    release_x = robot_path[-1][0] - release_retreat * math.cos(last_heading)
    release_y = robot_path[-1][1] - release_retreat * math.sin(last_heading)
    approach_candidates = _curve_candidates(
        (robot_x, robot_y), (staging_x, staging_y), keepout
    )
    valid_approaches = tuple(
        path
        for path in approach_candidates
        if polyline_outside_keepout(path, keepout)
    )
    if not valid_approaches:
        raise ValueError("no approach curve stays outside the remaining-cylinder keepout")
    approach_path = min(valid_approaches, key=path_length)
    return_candidates = _curve_candidates(
        (release_x, release_y), (home_x, home_y), keepout
    )
    valid_returns = tuple(
        (robot_path[-1],) + path
        for path in return_candidates
        if polyline_outside_keepout((robot_path[-1],) + path, keepout)
    )
    if not valid_returns:
        raise ValueError("no return curve stays outside the remaining-cylinder keepout")
    return_path = min(valid_returns, key=path_length)
    return PushPreviewPlan(
        selection=selection,
        keepout=keepout,
        home_x=home_x,
        home_y=home_y,
        home_yaw=home_yaw,
        destination_x=destination_x,
        destination_y=destination_y,
        staging_x=staging_x,
        staging_y=staging_y,
        staging_yaw=first_heading,
        approach_path=approach_path,
        target_path=target_path,
        robot_push_path=robot_path,
        release_x=release_x,
        release_y=release_y,
        return_path=return_path,
    )


def _curve_candidates(start, end, keepout: Keepout):
    direct = _quadratic_bezier(start, _midpoint(start, end), end)
    output = [direct]
    if keepout.hull:
        center = (
            sum(point[0] for point in keepout.hull) / len(keepout.hull),
            sum(point[1] for point in keepout.hull) / len(keepout.hull),
        )
        base_radius = max(math.dist(center, point) for point in keepout.hull)
        base_radius += keepout.clearance
        start_angle = math.atan2(start[1] - center[1], start[0] - center[0])
        for direction in (1.0, -1.0):
            for offset_degrees in (45.0, 60.0, 75.0, 90.0, 110.0):
                angle = start_angle + direction * math.radians(offset_degrees)
                for scale in (1.40, 1.75, 2.00, 2.50, 3.00):
                    control = (
                        center[0] + base_radius * scale * math.cos(angle),
                        center[1] + base_radius * scale * math.sin(angle),
                    )
                    output.append(_quadratic_bezier(start, control, end))
    return tuple(output)


def _quadratic_bezier(start, control, end, samples: int = 81):
    return tuple(
        (
            (1.0 - t) ** 2 * start[0]
            + 2.0 * (1.0 - t) * t * control[0]
            + t**2 * end[0],
            (1.0 - t) ** 2 * start[1]
            + 2.0 * (1.0 - t) * t * control[1]
            + t**2 * end[1],
        )
        for t in (index / (samples - 1) for index in range(samples))
    )


def _offset_behind_path(points, offset: float):
    output = []
    for index, point in enumerate(points):
        before = points[max(0, index - 1)]
        after = points[min(len(points) - 1, index + 1)]
        heading = math.atan2(after[1] - before[1], after[0] - before[0])
        output.append(
            (
                point[0] - offset * math.cos(heading),
                point[1] - offset * math.sin(heading),
            )
        )
    return tuple(output)


def _midpoint(first, second):
    return (0.5 * (first[0] + second[0]), 0.5 * (first[1] + second[1]))


def _distance_to_hull(point, hull) -> float:
    if len(hull) == 1:
        return math.dist(point, hull[0])
    if len(hull) >= 3 and _inside_convex(point, hull):
        return 0.0
    return min(
        _distance_to_segment(point, hull[index - 1], hull[index])
        for index in range(len(hull))
    )


def _inside_convex(point, hull) -> bool:
    return all(
        (hull[(i + 1) % len(hull)][0] - hull[i][0])
        * (point[1] - hull[i][1])
        - (hull[(i + 1) % len(hull)][1] - hull[i][1])
        * (point[0] - hull[i][0])
        >= -1e-9
        for i in range(len(hull))
    )


def _distance_to_segment(point, first, second) -> float:
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.dist(point, first)
    ratio = max(
        0.0,
        min(
            1.0,
            (
                (point[0] - first[0]) * dx
                + (point[1] - first[1]) * dy
            )
            / length_squared,
        ),
    )
    projection = (first[0] + ratio * dx, first[1] + ratio * dy)
    return math.dist(point, projection)


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
