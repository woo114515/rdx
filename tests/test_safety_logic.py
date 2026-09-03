import math

import pytest

from rdx_safety.safety_logic import (
    SafetyConfig,
    SafetyController,
    Scan,
    Velocity,
)


NOW = 10.0


def make_controller() -> SafetyController:
    return SafetyController(
        SafetyConfig(
            command_timeout=0.25,
            scan_timeout=0.30,
            max_linear_speed=0.18,
            max_angular_speed=0.60,
            stop_distance=0.35,
            slow_distance=0.60,
            sector_half_angle=math.radians(30.0),
            rotation_stop_distance=0.35,
        )
    )


def clear_scan(distance: float = 2.0) -> Scan:
    return Scan(
        angle_min=-math.pi,
        angle_increment=math.radians(1.0),
        ranges=(distance,) * 360,
        range_min=0.15,
        range_max=20.0,
    )


def scan_with_obstacle(angle: float, distance: float) -> Scan:
    ranges = list(clear_scan().ranges)
    index = round((angle + math.pi) / math.radians(1.0))
    ranges[index % len(ranges)] = distance
    return Scan(
        angle_min=-math.pi,
        angle_increment=math.radians(1.0),
        ranges=tuple(ranges),
        range_min=0.15,
        range_max=20.0,
    )


def evaluate(
    *,
    nav_command: Velocity | None = Velocity(0.10, 0.0, 0.0),
    nav_age: float = 0.0,
    teleop_command: Velocity | None = None,
    teleop_age: float = 0.0,
    scan: Scan | None = None,
    scan_age: float = 0.0,
    emergency_stop: bool = False,
    footprint_verified: bool = True,
):
    return make_controller().evaluate(
        now=NOW,
        nav_command=nav_command,
        nav_stamp=None if nav_command is None else NOW - nav_age,
        teleop_command=teleop_command,
        teleop_stamp=None if teleop_command is None else NOW - teleop_age,
        scan=clear_scan() if scan is None else scan,
        scan_stamp=NOW - scan_age,
        emergency_stop=emergency_stop,
        footprint_verified=footprint_verified,
    )


def test_unverified_footprint_blocks_motion():
    decision = evaluate(footprint_verified=False)

    assert decision.velocity == Velocity.zero()
    assert decision.reason == "footprint_unverified"
    assert decision.source == "none"


def test_emergency_stop_has_highest_priority():
    decision = evaluate(emergency_stop=True, footprint_verified=False)

    assert decision.velocity == Velocity.zero()
    assert decision.reason == "emergency_stop"


@pytest.mark.parametrize(
    ("nav_command", "nav_age", "expected_reason"),
    [
        (None, 0.0, "no_command"),
        (Velocity(0.1, 0.0, 0.0), 0.251, "command_timeout"),
        (Velocity(0.1, 0.0, 0.0), -0.001, "command_time_invalid"),
    ],
)
def test_missing_stale_or_future_command_stops(
    nav_command: Velocity | None,
    nav_age: float,
    expected_reason: str,
):
    decision = evaluate(nav_command=nav_command, nav_age=nav_age)

    assert decision.velocity == Velocity.zero()
    assert decision.reason == expected_reason


def test_zero_command_does_not_require_a_scan():
    decision = make_controller().evaluate(
        now=NOW,
        nav_command=Velocity.zero(),
        nav_stamp=NOW,
        teleop_command=None,
        teleop_stamp=None,
        scan=None,
        scan_stamp=None,
        emergency_stop=False,
        footprint_verified=True,
    )

    assert decision.velocity == Velocity.zero()
    assert decision.reason == "idle"
    assert decision.source == "nav"


@pytest.mark.parametrize(
    ("scan_value", "scan_age", "expected_reason"),
    [
        (None, 0.0, "scan_missing"),
        (clear_scan(), 0.301, "scan_timeout"),
    ],
)
def test_missing_or_stale_scan_stops(scan_value, scan_age, expected_reason):
    decision = make_controller().evaluate(
        now=NOW,
        nav_command=Velocity(0.1, 0.0, 0.0),
        nav_stamp=NOW,
        teleop_command=None,
        teleop_stamp=None,
        scan=scan_value,
        scan_stamp=None if scan_value is None else NOW - scan_age,
        emergency_stop=False,
        footprint_verified=True,
    )

    assert decision.velocity == Velocity.zero()
    assert decision.reason == expected_reason


def test_teleop_preempts_nav_and_velocity_is_limited():
    decision = evaluate(
        nav_command=Velocity(0.1, 0.0, 0.0),
        teleop_command=Velocity(1.0, -1.0, 2.0),
    )

    assert decision.source == "teleop"
    assert decision.velocity.linear_x == pytest.approx(0.18 / math.sqrt(2.0))
    assert decision.velocity.linear_y == pytest.approx(-0.18 / math.sqrt(2.0))
    assert decision.velocity.angular_z == pytest.approx(0.60)
    assert decision.reason == "clear"


@pytest.mark.parametrize(
    ("velocity", "obstacle_angle"),
    [
        (Velocity(0.10, 0.0, 0.0), 0.0),
        (Velocity(-0.10, 0.0, 0.0), math.pi),
        (Velocity(0.0, 0.10, 0.0), math.pi / 2.0),
        (Velocity(0.0, -0.10, 0.0), -math.pi / 2.0),
    ],
)
def test_close_obstacle_stops_in_translation_direction(velocity, obstacle_angle):
    decision = evaluate(
        nav_command=velocity,
        scan=scan_with_obstacle(obstacle_angle, 0.25),
    )

    assert decision.velocity == Velocity.zero()
    assert decision.reason == "obstacle_stop"


def test_obstacle_outside_motion_sector_does_not_stop_translation():
    decision = evaluate(
        nav_command=Velocity(0.10, 0.0, 0.0),
        scan=scan_with_obstacle(math.pi, 0.25),
    )

    assert decision.velocity == Velocity(0.10, 0.0, 0.0)
    assert decision.reason == "clear"


def test_mid_zone_obstacle_scales_the_whole_command():
    decision = evaluate(
        nav_command=Velocity(0.10, 0.0, 0.20),
        scan=scan_with_obstacle(0.0, 0.475),
    )

    assert decision.velocity.linear_x == pytest.approx(0.05)
    assert decision.velocity.linear_y == pytest.approx(0.0)
    assert decision.velocity.angular_z == pytest.approx(0.10)
    assert decision.reason == "obstacle_slow"


def test_close_obstacle_anywhere_stops_pure_rotation():
    decision = evaluate(
        nav_command=Velocity(0.0, 0.0, 0.30),
        scan=scan_with_obstacle(-2.0, 0.25),
    )

    assert decision.velocity == Velocity.zero()
    assert decision.reason == "obstacle_stop"


def test_infinite_ranges_mean_clear_and_nan_only_scan_is_invalid():
    infinite = clear_scan(distance=math.inf)
    clear_decision = evaluate(scan=infinite)
    invalid = Scan(
        angle_min=-math.pi,
        angle_increment=math.radians(1.0),
        ranges=(math.nan,) * 360,
        range_min=0.15,
        range_max=20.0,
    )
    invalid_decision = evaluate(scan=invalid)

    assert clear_decision.reason == "clear"
    assert invalid_decision.velocity == Velocity.zero()
    assert invalid_decision.reason == "scan_invalid"


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match="slow_distance"):
        SafetyConfig(stop_distance=0.6, slow_distance=0.35)


def test_nonfinite_velocity_is_rejected_explicitly():
    decision = evaluate(nav_command=Velocity(math.nan, 0.0, 0.0))

    assert decision.velocity == Velocity.zero()
    assert decision.reason == "command_invalid"
