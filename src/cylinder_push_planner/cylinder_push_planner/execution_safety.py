"""Pure safety checks for explicitly armed push-approach execution."""

from __future__ import annotations

import math
from typing import Any, Sequence


def unique_reacquisition_match(
    candidates: Sequence[tuple[float, float]],
    expected: tuple[float, float],
    maximum_correction: float,
    ambiguity_margin: float,
) -> tuple[tuple[float, float] | None, str]:
    """Select one near-range LiDAR candidate around a predicted target.

    The function deliberately compares two-dimensional centre positions, not
    only bearing or the first return in a forward corridor.  A match is
    accepted only when it is close enough to the locked target prediction and
    distinctly better than the runner-up.  Callers can therefore wait for
    another scan instead of silently choosing a neighbouring cylinder.
    """

    values = (*expected, maximum_correction, ambiguity_margin)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("reacquisition values must be finite")
    if maximum_correction <= 0.0 or ambiguity_margin < 0.0:
        raise ValueError("invalid reacquisition limits")
    if not candidates:
        return None, "no cylinder-sized LiDAR cluster"
    if not all(
        len(candidate) == 2
        and all(math.isfinite(value) for value in candidate)
        for candidate in candidates
    ):
        raise ValueError("reacquisition candidates must be finite 2D points")

    ranked = sorted(
        (math.dist(candidate, expected), candidate) for candidate in candidates
    )
    best_error, best = ranked[0]
    if best_error > maximum_correction:
        return None, f"nearest cluster is {best_error:.3f} m from prediction"
    if len(ranked) > 1:
        second_error = ranked[1][0]
        if second_error - best_error < ambiguity_margin:
            return None, (
                "ambiguous near-range clusters: "
                f"errors={best_error:.3f}/{second_error:.3f} m"
            )
    return best, f"unique cluster correction={best_error:.3f} m"


def match_reacquisition_reference(
    candidates: Sequence[tuple[float, float]],
    reference: tuple[float, float],
    observation_from_reference_translation: tuple[float, float],
    observation_from_reference_yaw: float,
    maximum_correction: float,
    ambiguity_margin: float,
) -> tuple[tuple[float, float] | None, tuple[float, float], str]:
    """Match a fixed-frame target in the current observation frame.

    A target copied into odometry when a snapshot is locked becomes stale when
    SLAM subsequently corrects ``map -> odom``.  Transforming the immutable
    fixed-frame reference at the observation timestamp keeps the prediction
    and raw candidates in one coherent frame.
    """

    tx, ty = observation_from_reference_translation
    values = (*reference, tx, ty, observation_from_reference_yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("reacquisition transform must be finite")
    cosine = math.cos(observation_from_reference_yaw)
    sine = math.sin(observation_from_reference_yaw)
    expected = (
        tx + cosine * reference[0] - sine * reference[1],
        ty + sine * reference[0] + cosine * reference[1],
    )
    match, reason = unique_reacquisition_match(
        candidates,
        expected,
        maximum_correction,
        ambiguity_margin,
    )
    return match, expected, reason


def nav2_path_result_error_code(result: Any) -> int:
    """Return a path result error code across Nav2 Humble variants.

    The robot's Humble ``ComputePathToPose`` result has no ``error_code``
    member. Newer Nav2 variants do, so honor it when present and otherwise
    rely on the action goal status.
    """

    return int(getattr(result, "error_code", 0))


def closest_path_index(
    path: Sequence[tuple[float, float]],
    point: tuple[float, float],
    first_index: int = 0,
) -> int:
    """Return the closest path pose without allowing progress to move backward."""

    if not path:
        raise ValueError("path cannot be empty")
    start = max(0, min(int(first_index), len(path) - 1))
    return min(range(start, len(path)), key=lambda index: math.dist(path[index], point))


def tracking_command(
    path: Sequence[tuple[float, float]],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    first_index: int,
    lookahead_distance: float,
    linear_speed: float,
    angular_gain: float,
    maximum_angular_speed: float,
) -> tuple[float, float, int, float]:
    """Return a bounded forward-only command for a sampled map-frame path."""

    if len(path) < 2:
        raise ValueError("tracking path needs at least two poses")
    if min(lookahead_distance, linear_speed, angular_gain, maximum_angular_speed) <= 0:
        raise ValueError("tracking parameters must be positive")
    nearest = closest_path_index(path, (robot_x, robot_y), first_index)
    target_index = nearest
    while (
        target_index + 1 < len(path)
        and math.dist((robot_x, robot_y), path[target_index]) < lookahead_distance
    ):
        target_index += 1
    target = path[target_index]
    desired = math.atan2(target[1] - robot_y, target[0] - robot_x)
    error = _normalize_angle(desired - robot_yaw)
    # Slow sharply when heading error grows; never reverse during a push.
    speed_scale = max(0.0, math.cos(error))
    linear = linear_speed * speed_scale
    angular = max(
        -maximum_angular_speed,
        min(maximum_angular_speed, angular_gain * error),
    )
    return linear, angular, nearest, error


def retreat_motion(
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    current_x: float,
    current_y: float,
    current_yaw: float,
) -> tuple[float, float, float]:
    """Return reverse progress, lateral drift and heading error from an origin."""

    values = (
        origin_x,
        origin_y,
        origin_yaw,
        current_x,
        current_y,
        current_yaw,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("retreat poses must be finite")
    dx = current_x - origin_x
    dy = current_y - origin_y
    reverse_progress = -(dx * math.cos(origin_yaw) + dy * math.sin(origin_yaw))
    lateral_drift = -dx * math.sin(origin_yaw) + dy * math.cos(origin_yaw)
    heading_error = _normalize_angle(current_yaw - origin_yaw)
    return reverse_progress, lateral_drift, heading_error


def retreat_pose_step(
    previous_x: float,
    previous_y: float,
    current_x: float,
    current_y: float,
) -> float:
    """Return one release-cycle translation, rejecting non-finite poses."""

    values = (previous_x, previous_y, current_x, current_y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("retreat poses must be finite")
    return math.hypot(current_x - previous_x, current_y - previous_y)


def odometry_step_is_plausible(
    previous_x: float,
    previous_y: float,
    current_x: float,
    current_y: float,
    elapsed: float,
    maximum_speed: float,
    distance_slack: float,
) -> bool:
    """Check an odometry step against elapsed time and a bounded speed.

    ``distance_slack`` covers encoder quantization and small estimator
    corrections. A longer callback interval therefore permits proportionally
    more real travel instead of being compared with a fixed per-tick limit.
    """

    values = (
        previous_x,
        previous_y,
        current_x,
        current_y,
        elapsed,
        maximum_speed,
        distance_slack,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry continuity values must be finite")
    if elapsed <= 0.0 or maximum_speed <= 0.0 or distance_slack < 0.0:
        raise ValueError("invalid odometry continuity limits")
    distance = math.hypot(current_x - previous_x, current_y - previous_y)
    return distance <= maximum_speed * elapsed + distance_slack


def front_target_present(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    maximum_distance: float,
    half_width: float,
) -> bool:
    """Return whether a valid LiDAR return occupies the narrow fork corridor."""

    if maximum_distance <= 0.0 or half_width <= 0.0:
        raise ValueError("target corridor dimensions must be positive")
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index * angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if 0.0 < x <= maximum_distance and abs(y) <= half_width:
            return True
    return False


def unexpected_obstacle_ahead(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    stop_distance: float,
    half_width: float,
    target_corridor_half_width: float,
    target_corridor_depth: float,
) -> bool:
    """Detect close frontal returns, excluding only the expected pushed target."""

    if min(stop_distance, half_width, target_corridor_half_width, target_corridor_depth) <= 0:
        raise ValueError("obstacle dimensions must be positive")
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index * angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if not (0.0 < x <= stop_distance and abs(y) <= half_width):
            continue
        expected_target = (
            x <= target_corridor_depth and abs(y) <= target_corridor_half_width
        )
        if not expected_target:
            return True
    return False


def obstacle_behind(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    stop_distance: float,
    half_width: float,
) -> bool:
    """Return whether a valid scan point blocks a short reverse release."""

    if stop_distance <= 0.0 or half_width <= 0.0:
        raise ValueError("reverse obstacle dimensions must be positive")
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + index * angle_increment
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        if -stop_distance <= x < 0.0 and abs(y) <= half_width:
            return True
    return False


def path_stays_outside_polygon(
    path: Sequence[tuple[float, float]],
    polygon: Sequence[tuple[float, float]],
    sample_spacing: float = 0.02,
) -> bool:
    """Return whether a sampled path avoids the interior of a polygon."""

    if sample_spacing <= 0.0:
        raise ValueError("sample_spacing must be positive")
    if len(path) < 2:
        return False
    if not polygon:
        return True
    if len(polygon) < 3:
        return False
    for start, end in zip(path, path[1:]):
        length = math.dist(start, end)
        steps = max(1, math.ceil(length / sample_spacing))
        for index in range(steps + 1):
            ratio = index / steps
            point = (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
            if not point_outside_polygon(point, polygon):
                return False
    return True


def point_outside_polygon(
    point: tuple[float, float],
    polygon: Sequence[tuple[float, float]],
) -> bool:
    """Return false for a point inside or on a valid polygon."""

    if not polygon:
        return True
    return len(polygon) >= 3 and not _point_in_polygon(point, polygon)


def approach_watchdog_fault(
    now: float,
    last_scan_received: float,
    navigation_started: float,
    scan_timeout: float,
    navigation_timeout: float,
) -> str | None:
    """Return the reason an active approach must be cancelled."""

    if scan_timeout <= 0.0 or navigation_timeout <= 0.0:
        raise ValueError("watchdog timeouts must be positive")
    if last_scan_received <= 0.0:
        return "laser scan is missing"
    if now - last_scan_received > scan_timeout:
        return "laser scan became stale"
    if now - navigation_started > navigation_timeout:
        return "approach navigation timed out"
    return None


def plan_inputs_fault(
    plan_ready: bool,
    snapshot_ready: bool,
    snapshot_locked: bool,
    has_staging_pose: bool,
    has_approach_path: bool,
    has_keepout: bool,
    has_push_path: bool = True,
    has_return_path: bool = True,
    has_target_path: bool = True,
) -> str | None:
    """Validate all immutable inputs before an approach can be armed."""

    if not plan_ready:
        return "push preview is not ready"
    if not snapshot_ready or not snapshot_locked:
        return "validated cylinder snapshot is not ready and locked"
    if not has_staging_pose:
        return "staging pose is missing"
    if not has_approach_path:
        return "approach preview path is missing"
    if not has_keepout:
        return "remaining-cylinder keepout is missing"
    if not has_push_path:
        return "robot push path is missing"
    if not has_return_path:
        return "return path is missing"
    if not has_target_path:
        return "target path is missing"
    return None


def _point_in_polygon(
    point: tuple[float, float],
    polygon: Sequence[tuple[float, float]],
) -> bool:
    """Return true for points inside or on the polygon boundary."""

    x, y = point
    inside = False
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        if _point_on_segment(point, first, second):
            return True
        x1, y1 = first
        x2, y2 = second
        if (y1 > y) == (y2 > y):
            continue
        crossing_x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        if crossing_x >= x:
            inside = not inside
    return inside


def _point_on_segment(point, first, second, tolerance: float = 1e-9) -> bool:
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    cross = dx * (point[1] - first[1]) - dy * (point[0] - first[0])
    if abs(cross) > tolerance:
        return False
    dot = (point[0] - first[0]) * dx + (point[1] - first[1]) * dy
    return -tolerance <= dot <= dx * dx + dy * dy + tolerance


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
