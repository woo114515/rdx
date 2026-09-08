from pathlib import Path
from types import SimpleNamespace

import yaml

from builtin_interfaces.msg import Time

from cylinder_push_planner.direct_task_node import DirectTaskControllerNode


class _ReacquisitionHarness:
    def __init__(self) -> None:
        self._scan = SimpleNamespace(
            header=SimpleNamespace(
                stamp=Time(sec=10, nanosec=20),
                frame_id="lidar_link",
            )
        )
        self._phase_started = 0.0
        self._reacquisition_reason = "waiting"
        self._reacquisition_scan_stamp = None
        self._reacquisition_reference_point = (1.0, 0.0)
        self._reacquisition_reference_frame = "map"
        self._plan = object()

    @staticmethod
    def _now() -> float:
        return 1.0

    @staticmethod
    def _float(name: str) -> float:
        if name == "target_reacquisition_timeout":
            return 2.0
        if name == "transform_timeout":
            return 0.15
        raise AssertionError(f"unexpected parameter: {name}")

    @staticmethod
    def _string(name: str) -> str:
        assert name == "execution_frame"
        return "odom"

    @staticmethod
    def _publish_zero() -> None:
        pass

    @staticmethod
    def _abort(reason: str) -> None:
        raise AssertionError(f"unexpected abort: {reason}")

    @staticmethod
    def _planar_transform(*args, **kwargs):
        del args, kwargs
        raise ValueError("timestamped transform is pending")


def test_reacquisition_reads_scan_before_timestamped_transform() -> None:
    harness = _ReacquisitionHarness()

    DirectTaskControllerNode._tick_target_reacquisition(
        harness,
        (0.0, 0.0, 0.0),
    )

    assert harness._reacquisition_scan_stamp == (10, 20)
    assert harness._reacquisition_reason == "timestamped transform is pending"


class _YawHarness:
    def __init__(self) -> None:
        self._heading_settle_cycles = 0
        self._phase_started = 0.0
        self.finalized = False

    @staticmethod
    def _now() -> float:
        return 1.0

    @staticmethod
    def _float(name: str) -> float:
        return {"heading_tolerance": 0.08, "alignment_timeout": 15.0}[name]

    @staticmethod
    def _bool(name: str) -> bool:
        assert name == "inertial_heading_enabled"
        return False

    @staticmethod
    def _publish_zero() -> None:
        pass

    @staticmethod
    def _publish_command(linear: float, angular: float) -> None:
        del linear, angular

    def _begin_finalize(self) -> None:
        self.finalized = True

    @staticmethod
    def _abort(reason: str) -> None:
        raise AssertionError(f"unexpected abort: {reason}")


def test_home_yaw_does_not_finalize_until_absolute_heading_is_stable() -> None:
    harness = _YawHarness()

    DirectTaskControllerNode._tick_yaw(
        harness, (0.0, 0.0, 0.01), 0.0, heading_stable=False
    )
    assert not harness.finalized

    DirectTaskControllerNode._tick_yaw(
        harness, (0.0, 0.0, 0.01), 0.0, heading_stable=True
    )
    assert harness.finalized


class _ReleaseHarness:
    def __init__(self) -> None:
        self._phase_started = 0.0
        self._release_origin = (0.0, 0.0, 0.0)
        self._tf_failures = 0
        self.started = None

    @staticmethod
    def _now() -> float:
        return 1.0

    @staticmethod
    def _float(name: str) -> float:
        return {
            "release_timeout": 10.0,
            "release_distance": 0.15,
            "release_speed": 0.15,
            "minimum_linear_speed": 0.15,
        }[name]

    @staticmethod
    def _int(name: str) -> int:
        assert name == "transform_failure_limit"
        return 3

    @staticmethod
    def _bool(name: str) -> bool:
        if name == "release_forward_motion_guard_enabled":
            return False
        raise AssertionError(f"unexpected parameter: {name}")

    @staticmethod
    def _string(name: str) -> str:
        assert name == "odom_frame"
        return "odom"

    @staticmethod
    def _live_obstacle_stop_enabled() -> bool:
        return False

    @staticmethod
    def _robot_pose_in_frame(frame: str):
        assert frame == "odom"
        return (-0.16, 0.0, 0.0)

    @staticmethod
    def _publish_zero() -> None:
        pass

    @staticmethod
    def _publish_command(linear: float, angular: float) -> None:
        raise AssertionError(f"unexpected motion command: {linear}, {angular}")

    def _start_phase(self, state: str, start) -> None:
        self.started = state, start

    @staticmethod
    def _abort(reason: str) -> None:
        raise AssertionError(f"unexpected abort: {reason}")


def test_release_distance_uses_odom_even_when_execution_pose_disagrees() -> None:
    harness = _ReleaseHarness()

    DirectTaskControllerNode._tick_release(harness, (99.0, 99.0, 2.0))

    assert harness.started == ("aligning_return", (99.0, 99.0))


def test_deployed_direct_task_tracks_map_geometry_in_map() -> None:
    config = Path(__file__).parents[1] / "config" / "direct_task.yaml"
    parameters = yaml.safe_load(config.read_text(encoding="utf-8"))[
        "cylinder_direct_task_controller"
    ]["ros__parameters"]

    assert parameters["fixed_frame"] == "map"
    assert parameters["execution_frame"] == "map"
    assert parameters["odom_frame"] == "odom"
