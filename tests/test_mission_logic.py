import math
from pathlib import Path

import pytest
import yaml

from rdx_mission.mission_logic import (
    MissionConfigError,
    MissionStateMachine,
    quaternion_from_yaw,
    load_mission,
)


EXPECTED_NAMES = ["task_1", "task_2", "task_3", "start"]


def plan_data(*, ready=True, names=None):
    names = EXPECTED_NAMES if names is None else names
    return {
        "mission": {
            "ready": ready,
            "frame_id": "map",
            "waypoints": [
                {
                    "name": name,
                    "x": float(index),
                    "y": float(index) / 2.0,
                    "yaw": 0.1 * index,
                }
                for index, name in enumerate(names)
            ],
        }
    }


def write_plan(tmp_path: Path, data: dict) -> Path:
    path = tmp_path / "waypoints.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def test_loads_exact_competition_order(tmp_path):
    plan = load_mission(write_plan(tmp_path, plan_data()))

    assert [point.name for point in plan.waypoints] == EXPECTED_NAMES
    assert plan.frame_id == "map"
    assert plan.waypoints[1].x == pytest.approx(1.0)


@pytest.mark.parametrize(
    "data",
    [
        plan_data(ready=False),
        plan_data(names=["task_1", "task_2", "task_2", "start"]),
        plan_data(names=["task_2", "task_1", "task_3", "start"]),
    ],
)
def test_unready_duplicate_or_wrong_order_plan_is_rejected(tmp_path, data):
    with pytest.raises(MissionConfigError):
        load_mission(write_plan(tmp_path, data))


def test_wrong_count_or_nonfinite_pose_is_rejected(tmp_path):
    too_few = plan_data()
    too_few["mission"]["waypoints"].pop()
    with pytest.raises(MissionConfigError, match="exactly"):
        load_mission(write_plan(tmp_path, too_few))

    nonfinite = plan_data()
    nonfinite["mission"]["waypoints"][0]["x"] = float("nan")
    with pytest.raises(MissionConfigError, match="finite"):
        load_mission(write_plan(tmp_path, nonfinite))


def test_non_string_waypoint_name_is_rejected(tmp_path):
    malformed = plan_data()
    malformed["mission"]["waypoints"][0]["name"] = ["task_1"]

    with pytest.raises(MissionConfigError, match="name"):
        load_mission(write_plan(tmp_path, malformed))


def test_failure_does_not_skip_to_next_waypoint(tmp_path):
    machine = MissionStateMachine(load_mission(write_plan(tmp_path, plan_data())))

    assert machine.start().name == "task_1"
    machine.goal_failed("blocked")

    assert machine.state == "failed"
    assert machine.next_goal() is None


def test_success_returns_to_start_and_completes(tmp_path):
    machine = MissionStateMachine(load_mission(write_plan(tmp_path, plan_data())))

    visited = [machine.start().name]
    while machine.state == "running":
        next_goal = machine.goal_succeeded()
        if next_goal is not None:
            visited.append(next_goal.name)

    assert visited == EXPECTED_NAMES
    assert machine.state == "completed"


def test_cancel_stops_mission_without_advancing(tmp_path):
    machine = MissionStateMachine(load_mission(write_plan(tmp_path, plan_data())))

    machine.start()
    machine.cancel()

    assert machine.state == "cancelled"
    assert machine.next_goal() is None


def test_machine_cannot_start_twice(tmp_path):
    machine = MissionStateMachine(load_mission(write_plan(tmp_path, plan_data())))

    machine.start()
    with pytest.raises(RuntimeError, match="running"):
        machine.start()


def test_quaternion_from_yaw_is_planar_and_normalized():
    x, y, z, w = quaternion_from_yaw(1.2)

    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert z == pytest.approx(math.sin(0.6))
    assert w == pytest.approx(math.cos(0.6))
