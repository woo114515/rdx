"""Pure geometry for the compact, non-replanning Task 3 push cycle."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .geometry import (
    SelectionPlan,
    Target,
    build_push_preview,
    minimum_enclosing_circle,
    select_right_first,
)


Point2D = tuple[float, float]


@dataclass(frozen=True)
class DirectCyclePlan:
    """One target and the prevalidated paths used by the compact controller."""

    selection: SelectionPlan
    home: tuple[float, float, float]
    field_center: Point2D
    destination: Point2D
    staging: Point2D
    contact: Point2D
    push_end: Point2D
    release_end: Point2D
    approach_path: tuple[Point2D, ...]
    contact_path: tuple[Point2D, Point2D]
    push_path: tuple[Point2D, ...]
    return_path: tuple[Point2D, ...]
    keepout_boundary: tuple[Point2D, ...]


def build_direct_cycle(
    targets: Sequence[Target],
    robot_pose: tuple[float, float, float],
    destination: Point2D,
    staging_clearance: float,
    contact_offset: float,
    release_distance: float,
    transit_clearance: float,
    *,
    selection: SelectionPlan | None = None,
    home: tuple[float, float, float] | None = None,
    field_center: Point2D | None = None,
    require_clear_corridors: bool = True,
    allow_local_approach_inside_keepout: bool = False,
) -> DirectCyclePlan:
    """Build a compact push cycle, routing around the remaining-cylinder hull."""

    if not targets:
        raise ValueError("at least one validated target is required")
    values = (*robot_pose, *destination)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("robot pose and destination must be finite")
    if min(staging_clearance, contact_offset, release_distance) <= 0.0:
        raise ValueError("cycle distances must be positive")
    if transit_clearance < 0.0:
        raise ValueError("transit clearance must not be negative")

    robot_x, robot_y, _robot_yaw = robot_pose
    if selection is None:
        selection = select_right_first(
            targets, robot_x, robot_y, staging_clearance
        )
    if selection.target not in targets:
        raise ValueError("selection target must belong to targets")
    if home is None:
        home = robot_pose
    if field_center is None:
        circle = minimum_enclosing_circle([(item.x, item.y) for item in targets])
        assert circle is not None
        field_center = (circle.x, circle.y)

    if require_clear_corridors:
        # The selected target is excluded by build_push_preview before the
        # keepout is formed. The clearance is measured from the remaining
        # cylinder centres, so include their physical radius as well as the
        # configured robot-centre transit clearance.
        robot_clearance = transit_clearance + max(item.radius for item in targets)
        preview = build_push_preview(
            targets,
            robot_x,
            robot_y,
            robot_pose[2],
            destination[0],
            destination[1],
            staging_clearance,
            robot_clearance,
            contact_offset,
            release_distance,
            selection=selection,
            task_home=home,
            allow_local_approach_inside_keepout=(
                allow_local_approach_inside_keepout
            ),
        )
        contact = preview.robot_push_path[0]
        push_end = preview.robot_push_path[-1]
        release_end = (preview.release_x, preview.release_y)
        return DirectCyclePlan(
            selection=selection,
            home=home,
            field_center=field_center,
            destination=destination,
            staging=(preview.staging_x, preview.staging_y),
            contact=contact,
            push_end=push_end,
            release_end=release_end,
            approach_path=preview.approach_path,
            contact_path=((preview.staging_x, preview.staging_y), contact),
            push_path=preview.robot_push_path,
            return_path=preview.return_path,
            keepout_boundary=preview.keepout.boundary,
        )

    target = selection.target
    direction_x = destination[0] - target.x
    direction_y = destination[1] - target.y
    distance = math.hypot(direction_x, direction_y)
    if distance < 0.25:
        raise ValueError("destination must be separated from the selected target")
    direction_x /= distance
    direction_y /= distance

    contact = (
        target.x - contact_offset * direction_x,
        target.y - contact_offset * direction_y,
    )
    staging = (
        contact[0] - staging_clearance * direction_x,
        contact[1] - staging_clearance * direction_y,
    )
    push_end = (
        destination[0] - contact_offset * direction_x,
        destination[1] - contact_offset * direction_y,
    )
    release_end = (
        push_end[0] - release_distance * direction_x,
        push_end[1] - release_distance * direction_y,
    )
    approach = ((robot_x, robot_y), staging)
    contact_path = (staging, contact)
    push_path = (contact, push_end)
    # Return over the exact corridors already checked for the outbound motion.
    # A direct chord from the delivery zone to home commonly cuts through the
    # cylinder group even though every outbound segment was clear.
    return_path = (release_end, contact, staging, (home[0], home[1]))

    return DirectCyclePlan(
        selection=selection,
        home=home,
        field_center=field_center,
        destination=destination,
        staging=staging,
        contact=contact,
        push_end=push_end,
        release_end=release_end,
        approach_path=approach,
        contact_path=contact_path,
        push_path=push_path,
        return_path=return_path,
        keepout_boundary=(),
    )


def stitch_reacquired_return_path(
    corrected_return: Sequence[Point2D],
    previous_approach: Sequence[Point2D],
) -> tuple[Point2D, ...]:
    """Return via the corrected local segment, then retrace the proven approach."""

    if not corrected_return or not previous_approach:
        raise ValueError("return and previous approach paths must not be empty")
    combined = tuple(corrected_return) + tuple(reversed(previous_approach))
    output = []
    for point in combined:
        if not output or math.dist(output[-1], point) > 1e-9:
            output.append(point)
    return tuple(output)


def first_corridor_blocker(
    segment: tuple[Point2D, Point2D],
    targets: Sequence[Target],
    clearance: float,
) -> Target | None:
    """Return the first cylinder intersecting a swept robot-centre segment."""

    if clearance < 0.0:
        raise ValueError("clearance must not be negative")
    for target in targets:
        if distance_to_segment((target.x, target.y), *segment) <= (
            clearance + target.radius
        ):
            return target
    return None


def distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    """Return Euclidean distance from a point to a finite segment."""

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    squared = dx * dx + dy * dy
    if squared <= 1e-18:
        return math.dist(point, start)
    ratio = (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / squared
    ratio = max(0.0, min(1.0, ratio))
    projection = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.dist(point, projection)


def polyline_tracking_target(
    path: Sequence[Point2D], position: Point2D, lookahead: float
) -> tuple[Point2D, float]:
    """Return a lookahead point from the nearest progress on a polyline.

    The nearest projection, rather than proximity to a discrete waypoint, is
    used so passing a densely sampled curve point cannot make the controller
    turn backwards to chase it.
    """

    if len(path) < 2:
        raise ValueError("path must contain at least two points")
    if lookahead <= 0.0 or not math.isfinite(lookahead):
        raise ValueError("lookahead must be positive and finite")
    if not all(math.isfinite(value) for point in (*path, position) for value in point):
        raise ValueError("path and position values must be finite")

    best_distance = math.inf
    best_index = 0
    best_projection = path[0]
    for index, (start, end) in enumerate(zip(path, path[1:])):
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
        # Prefer later progress when adjacent segments share the same vertex.
        if distance <= best_distance:
            best_distance = distance
            best_index = index
            best_projection = projection

    remaining = lookahead
    cursor = best_projection
    for endpoint in path[best_index + 1 :]:
        length = math.dist(cursor, endpoint)
        if length >= remaining and length > 1e-18:
            ratio = remaining / length
            return (
                cursor[0] + ratio * (endpoint[0] - cursor[0]),
                cursor[1] + ratio * (endpoint[1] - cursor[1]),
            ), best_distance
        remaining -= length
        cursor = endpoint
    return path[-1], best_distance


def heading_error(current: float, desired: float) -> float:
    """Return the shortest signed heading error."""

    return math.atan2(math.sin(desired - current), math.cos(desired - current))


def transform_point_2d(
    point: Point2D,
    translation: Point2D,
    yaw: float,
) -> Point2D:
    """Apply a planar rigid transform to a point."""

    values = (*point, *translation, yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("planar transform values must be finite")
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        translation[0] + cosine * point[0] - sine * point[1],
        translation[1] + sine * point[0] + cosine * point[1],
    )


def enforce_minimum_linear_speed(command: float, minimum: float) -> float:
    """Keep non-zero drive commands outside the base motor deadband.

    A true stop must remain exactly zero.  Otherwise preserve the requested
    direction and raise only the magnitude that falls below ``minimum``.
    """

    if not math.isfinite(command) or not math.isfinite(minimum):
        raise ValueError("linear speed values must be finite")
    if minimum < 0.0:
        raise ValueError("minimum linear speed must not be negative")
    if command == 0.0:
        return 0.0
    return math.copysign(max(abs(command), minimum), command)


def pure_pursuit_command(
    requested_speed: float,
    minimum_speed: float,
    heading_error_value: float,
    target_distance: float,
    heading_limit: float,
    maximum_angular_speed: float,
) -> tuple[float, float]:
    """Return a smooth differential-drive command for a lookahead target."""

    values = (
        requested_speed,
        minimum_speed,
        heading_error_value,
        target_distance,
        heading_limit,
        maximum_angular_speed,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pure-pursuit values must be finite")
    if requested_speed <= 0.0 or minimum_speed < 0.0:
        raise ValueError("pure-pursuit linear speeds are invalid")
    if target_distance <= 0.0 or heading_limit <= 0.0:
        raise ValueError("target distance and heading limit must be positive")
    if maximum_angular_speed <= 0.0:
        raise ValueError("maximum angular speed must be positive")

    error = math.atan2(
        math.sin(heading_error_value), math.cos(heading_error_value)
    )
    if abs(error) >= heading_limit:
        return 0.0, math.copysign(maximum_angular_speed, error)

    curvature = 2.0 * math.sin(error) / target_distance
    if abs(curvature) > maximum_angular_speed / max(minimum_speed, 1e-9):
        return 0.0, math.copysign(maximum_angular_speed, curvature)

    linear = requested_speed
    if abs(curvature) > 1e-9:
        linear = min(linear, maximum_angular_speed / abs(curvature))
    linear = enforce_minimum_linear_speed(linear, minimum_speed)
    angular = linear * curvature
    angular = max(-maximum_angular_speed, min(maximum_angular_speed, angular))
    return linear, angular
