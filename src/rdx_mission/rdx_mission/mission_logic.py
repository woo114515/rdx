"""ROS-independent waypoint validation and mission state machine."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Optional, Tuple

import yaml


EXPECTED_WAYPOINT_NAMES = ("task_1", "task_2", "task_3", "start")


class MissionConfigError(ValueError):
    """Raised when the saved waypoint configuration is unsafe or malformed."""


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return a normalized planar quaternion for a map-frame yaw."""

    if not math.isfinite(yaw):
        raise ValueError("yaw must be finite")
    half_yaw = yaw / 2.0
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


@dataclass(frozen=True)
class Waypoint:
    """A navigation goal in the mission frame."""

    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MissionPlan:
    """Validated fixed-order plan."""

    frame_id: str
    waypoints: Tuple[Waypoint, ...]


def load_mission(path: str | Path) -> MissionPlan:
    """Load a ready, finite, exactly ordered mission plan from YAML."""

    path = Path(path)
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise MissionConfigError(f"cannot read mission file {path}: {error}") from error

    if not isinstance(data, dict) or not isinstance(data.get("mission"), dict):
        raise MissionConfigError("mission section is required")
    mission = data["mission"]
    if mission.get("ready") is not True:
        raise MissionConfigError("mission is not ready; record real RViz poses first")

    frame_id = mission.get("frame_id")
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise MissionConfigError("frame_id must be a non-empty string")

    entries = mission.get("waypoints")
    if not isinstance(entries, list) or len(entries) != len(EXPECTED_WAYPOINT_NAMES):
        raise MissionConfigError(
            f"waypoints must contain exactly {len(EXPECTED_WAYPOINT_NAMES)} entries"
        )

    names = [
        entry.get("name") if isinstance(entry, dict) else None for entry in entries
    ]
    if not all(isinstance(name, str) and name.strip() for name in names):
        raise MissionConfigError("each waypoint name must be a non-empty string")
    if len(set(names)) != len(names):
        raise MissionConfigError("waypoint names contain a duplicate")
    if tuple(names) != EXPECTED_WAYPOINT_NAMES:
        raise MissionConfigError(
            "waypoint order must be task_1, task_2, task_3, start"
        )

    waypoints = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise MissionConfigError("each waypoint must be a mapping")
        values = (entry.get("x"), entry.get("y"), entry.get("yaw"))
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            raise MissionConfigError(
                f"waypoint {entry['name']} pose values must be finite numbers"
            )
        waypoints.append(
            Waypoint(
                name=entry["name"],
                x=float(entry["x"]),
                y=float(entry["y"]),
                yaw=float(entry["yaw"]),
            )
        )

    return MissionPlan(frame_id=frame_id.strip(), waypoints=tuple(waypoints))


class MissionStateMachine:
    """Advance through goals only after an explicit successful result."""

    def __init__(self, plan: MissionPlan) -> None:
        self._plan = plan
        self._index = -1
        self.state = "idle"
        self.failure_reason: Optional[str] = None

    @property
    def plan(self) -> MissionPlan:
        return self._plan

    @property
    def current(self) -> Optional[Waypoint]:
        if self.state != "running" or self._index < 0:
            return None
        return self._plan.waypoints[self._index]

    def start(self) -> Waypoint:
        if self.state != "idle":
            raise RuntimeError(f"mission is already {self.state}")
        self._index = 0
        self.state = "running"
        self.failure_reason = None
        return self._plan.waypoints[self._index]

    def next_goal(self) -> Optional[Waypoint]:
        return self.current

    def goal_succeeded(self) -> Optional[Waypoint]:
        if self.state != "running":
            return None
        if self._index == len(self._plan.waypoints) - 1:
            self.state = "completed"
            return None
        self._index += 1
        return self._plan.waypoints[self._index]

    def goal_failed(self, reason: str) -> None:
        if self.state == "running":
            self.state = "failed"
            self.failure_reason = reason or "navigation action failed"

    def cancel(self) -> None:
        if self.state == "running":
            self.state = "cancelled"
