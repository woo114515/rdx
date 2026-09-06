"""Pure velocity safety decisions with no ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


def heartbeat_is_stale(
    now: float, stamp: Optional[float], timeout: float
) -> bool:
    """Return true when a required heartbeat is absent or outside its TTL."""
    if not math.isfinite(now) or not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError("now and timeout must be finite; timeout must be positive")
    if stamp is None or not math.isfinite(stamp):
        return True
    age = now - stamp
    return age < 0.0 or age > timeout


@dataclass(frozen=True)
class Velocity:
    """Planar robot velocity in the base frame."""

    linear_x: float
    linear_y: float
    angular_z: float

    @classmethod
    def zero(cls) -> "Velocity":
        return cls(0.0, 0.0, 0.0)

    def is_zero(self, tolerance: float = 1e-9) -> bool:
        return (
            abs(self.linear_x) <= tolerance
            and abs(self.linear_y) <= tolerance
            and abs(self.angular_z) <= tolerance
        )


@dataclass(frozen=True)
class Scan:
    """Minimal LaserScan representation used by the safety core."""

    angle_min: float
    angle_increment: float
    ranges: Tuple[float, ...]
    range_min: float
    range_max: float

    def __init__(
        self,
        angle_min: float,
        angle_increment: float,
        ranges: Sequence[float],
        range_min: float,
        range_max: float,
    ) -> None:
        object.__setattr__(self, "angle_min", float(angle_min))
        object.__setattr__(self, "angle_increment", float(angle_increment))
        object.__setattr__(self, "ranges", tuple(float(value) for value in ranges))
        object.__setattr__(self, "range_min", float(range_min))
        object.__setattr__(self, "range_max", float(range_max))


@dataclass(frozen=True)
class SafetyConfig:
    """Thresholds for command selection and obstacle braking."""

    command_timeout: float = 0.25
    scan_timeout: float = 0.30
    max_linear_speed: float = 0.18
    max_angular_speed: float = 0.60
    stop_distance: float = 0.35
    slow_distance: float = 0.60
    sector_half_angle: float = math.radians(30.0)
    rotation_stop_distance: float = 0.35

    def __post_init__(self) -> None:
        positive = {
            "command_timeout": self.command_timeout,
            "scan_timeout": self.scan_timeout,
            "max_linear_speed": self.max_linear_speed,
            "max_angular_speed": self.max_angular_speed,
            "stop_distance": self.stop_distance,
            "slow_distance": self.slow_distance,
            "sector_half_angle": self.sector_half_angle,
            "rotation_stop_distance": self.rotation_stop_distance,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if self.slow_distance <= self.stop_distance:
            raise ValueError("slow_distance must be greater than stop_distance")
        if self.sector_half_angle > math.pi:
            raise ValueError("sector_half_angle must not exceed pi")


@dataclass(frozen=True)
class SafetyDecision:
    """Safe output plus a machine-readable diagnostic reason."""

    velocity: Velocity
    reason: str
    source: str
    clearance: Optional[float] = None


class SafetyController:
    """Select, limit, and obstacle-check velocity requests."""

    def __init__(self, config: SafetyConfig) -> None:
        self._config = config

    def evaluate(
        self,
        *,
        now: float,
        nav_command: Optional[Velocity],
        nav_stamp: Optional[float],
        teleop_command: Optional[Velocity],
        teleop_stamp: Optional[float],
        scan: Optional[Scan],
        scan_stamp: Optional[float],
        emergency_stop: bool,
        footprint_verified: bool,
    ) -> SafetyDecision:
        if emergency_stop:
            return self._stopped("emergency_stop")
        if not footprint_verified:
            return self._stopped("footprint_unverified")

        selection = self._select_command(
            now=now,
            nav_command=nav_command,
            nav_stamp=nav_stamp,
            teleop_command=teleop_command,
            teleop_stamp=teleop_stamp,
        )
        if isinstance(selection, SafetyDecision):
            return selection

        command, source = selection
        if not self._velocity_is_finite(command):
            return self._stopped("command_invalid", source)
        limited = self._limit(command)
        if limited.is_zero():
            return SafetyDecision(limited, "idle", source)

        scan_error = self._scan_error(now, scan, scan_stamp)
        if scan_error is not None:
            return self._stopped(scan_error, source)
        assert scan is not None

        clearance = self._clearance(scan, limited)
        if clearance is None:
            return self._stopped("scan_invalid", source)

        stop_distance = (
            self._config.rotation_stop_distance
            if self._is_pure_rotation(limited)
            else self._config.stop_distance
        )
        if clearance <= stop_distance:
            return SafetyDecision(
                Velocity.zero(), "obstacle_stop", source, clearance
            )

        if clearance < self._config.slow_distance:
            scale = (clearance - stop_distance) / (
                self._config.slow_distance - stop_distance
            )
            scale = min(1.0, max(0.0, scale))
            return SafetyDecision(
                Velocity(
                    limited.linear_x * scale,
                    limited.linear_y * scale,
                    limited.angular_z * scale,
                ),
                "obstacle_slow",
                source,
                clearance,
            )

        return SafetyDecision(limited, "clear", source, clearance)

    def _select_command(
        self,
        *,
        now: float,
        nav_command: Optional[Velocity],
        nav_stamp: Optional[float],
        teleop_command: Optional[Velocity],
        teleop_stamp: Optional[float],
    ):
        present = False
        stale = False
        for source, command, stamp in (
            ("teleop", teleop_command, teleop_stamp),
            ("nav", nav_command, nav_stamp),
        ):
            if command is None:
                continue
            present = True
            if stamp is None or not math.isfinite(stamp):
                return self._stopped("command_time_invalid", source)
            age = now - stamp
            if age < 0.0:
                return self._stopped("command_time_invalid", source)
            if age <= self._config.command_timeout:
                return command, source
            stale = True

        if stale:
            return self._stopped("command_timeout")
        if not present:
            return self._stopped("no_command")
        return self._stopped("command_timeout")

    def _scan_error(
        self,
        now: float,
        scan: Optional[Scan],
        scan_stamp: Optional[float],
    ) -> Optional[str]:
        if scan is None or scan_stamp is None:
            return "scan_missing"
        if not math.isfinite(scan_stamp) or now - scan_stamp < 0.0:
            return "scan_time_invalid"
        if now - scan_stamp > self._config.scan_timeout:
            return "scan_timeout"
        if (
            not scan.ranges
            or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_max <= 0.0
            or scan.range_max <= scan.range_min
        ):
            return "scan_invalid"
        return None

    def _limit(self, command: Velocity) -> Velocity:
        magnitude = math.hypot(command.linear_x, command.linear_y)
        linear_scale = (
            min(1.0, self._config.max_linear_speed / magnitude)
            if magnitude > 0.0
            else 1.0
        )
        angular = min(
            self._config.max_angular_speed,
            max(-self._config.max_angular_speed, command.angular_z),
        )
        return Velocity(
            command.linear_x * linear_scale,
            command.linear_y * linear_scale,
            angular,
        )

    @staticmethod
    def _velocity_is_finite(command: Velocity) -> bool:
        return all(
            math.isfinite(value)
            for value in (
                command.linear_x,
                command.linear_y,
                command.angular_z,
            )
        )

    def _clearance(self, scan: Scan, command: Velocity) -> Optional[float]:
        pure_rotation = self._is_pure_rotation(command)
        direction = math.atan2(command.linear_y, command.linear_x)
        valid = []

        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if not pure_rotation and self._angle_distance(angle, direction) > (
                self._config.sector_half_angle
            ):
                continue
            normalized = self._normalize_range(value, scan)
            if normalized is not None:
                valid.append(normalized)

        return min(valid) if valid else None

    @staticmethod
    def _normalize_range(value: float, scan: Scan) -> Optional[float]:
        if math.isnan(value) or value <= 0.0:
            # Some lidars encode an object closer than range_min as zero or
            # NaN. In the active motion sector, fail closed instead of
            # treating that return as free space.
            return 0.0
        if math.isinf(value) or value > scan.range_max:
            return scan.range_max
        return value

    @staticmethod
    def _is_pure_rotation(command: Velocity) -> bool:
        return math.hypot(command.linear_x, command.linear_y) <= 1e-9

    @staticmethod
    def _angle_distance(first: float, second: float) -> float:
        return abs((first - second + math.pi) % (2.0 * math.pi) - math.pi)

    @staticmethod
    def _stopped(reason: str, source: str = "none") -> SafetyDecision:
        return SafetyDecision(Velocity.zero(), reason, source)
