from pathlib import Path

import pytest

from cylinder_push_planner.task_workflow import (
    RUN_ONCE_COMPLETE_STATES,
    advance_services,
    automatic_cycle_ready,
    run_once_service,
    snapshot_matches_collection,
    validate_inventory,
)


def test_orchestrator_does_not_shadow_rclpy_client_storage():
    source_path = (
        Path(__file__).parents[1]
        / "cylinder_push_planner"
        / "task_orchestrator_node.py"
    )
    source = source_path.read_text(encoding="utf-8")

    assert "self._clients" not in source


@pytest.mark.parametrize(
    ("state", "stage"),
    [
        ("idle", "approach"),
        ("approach_complete", "contact"),
        ("contact_complete", "push"),
        ("push_complete", "return"),
        ("return_complete", "finalize"),
    ],
)
def test_advance_services_preserves_physical_stage_boundaries(state, stage):
    assert advance_services(state)[2] == stage


def test_advance_services_rejects_active_motion_state():
    with pytest.raises(ValueError, match="cannot advance"):
        advance_services("pushing")


@pytest.mark.parametrize(
    ("state", "service"),
    [
        ("idle", "/cylinder_push_execution/arm"),
        ("armed", "/cylinder_push_execution/start_approach"),
        ("approach_complete", "/cylinder_push_execution/arm_contact"),
        ("contact_armed", "/cylinder_push_execution/start_contact"),
        ("contact_complete", "/cylinder_push_execution/arm_push"),
        ("push_armed", "/cylinder_push_execution/start_push"),
        ("push_complete", "/cylinder_push_execution/arm_return"),
        ("return_armed", "/cylinder_push_execution/start_return"),
        ("return_complete", "/cylinder_push_execution/finalize_delivery"),
    ],
)
def test_run_once_service_advances_at_verified_boundaries(state, service):
    assert run_once_service(state)[0] == service


@pytest.mark.parametrize(
    "state",
    ["preflight", "approaching", "contacting", "pushing", "returning"],
)
def test_run_once_service_waits_during_active_execution(state):
    assert run_once_service(state) is None


def test_run_once_completion_states_do_not_request_another_service():
    assert RUN_ONCE_COMPLETE_STATES == {"collecting_remaining", "task_complete"}
    assert all(run_once_service(state) is None for state in RUN_ONCE_COMPLETE_STATES)


def test_run_once_rejects_fault_state_instead_of_retrying_motion():
    with pytest.raises(ValueError, match="cannot continue"):
        run_once_service("failed")


def test_automatic_cycle_requires_locked_ready_snapshot_while_collecting():
    assert automatic_cycle_ready("collecting", True, True, False)
    assert automatic_cycle_ready("collecting_remaining", True, True, False)
    assert not automatic_cycle_ready("collecting", False, True, False)
    assert not automatic_cycle_ready("collecting", True, False, False)
    assert not automatic_cycle_ready("collecting", True, True, True)
    assert not automatic_cycle_ready("failed", True, True, False)


def test_inventory_accepts_configurable_colors_and_counts():
    validate_inventory(["blue", "green", "red", "yellow"], [2, 2, 2, 1])


@pytest.mark.parametrize(
    ("colors", "counts"),
    [([], []), (["red"], [0]), (["red", "red"], [1, 1]), (["red"], [1, 2])],
)
def test_inventory_rejects_invalid_configuration(colors, counts):
    with pytest.raises(ValueError):
        validate_inventory(colors, counts)


def test_snapshot_must_match_current_inventory_and_collection_time():
    assert snapshot_matches_collection(
        ["blue", "green", "red"],
        [2, 1, 2],
        ["blue", "green", "red"],
        [2, 1, 2],
        101.0,
        100.0,
    )
    assert not snapshot_matches_collection(
        ["blue", "green", "red"],
        [2, 1, 2],
        ["blue", "green", "red"],
        [2, 2, 2],
        101.0,
        100.0,
    )
    assert not snapshot_matches_collection(
        ["blue", "green", "red"],
        [2, 1, 2],
        ["blue", "green", "red"],
        [2, 1, 2],
        99.9,
        100.0,
    )
