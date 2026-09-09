"""Pure mappings used by the Task 3 service orchestrator."""

from __future__ import annotations


ADVANCE_SERVICES = {
    "idle": (
        "/cylinder_push_execution/arm",
        "/cylinder_push_execution/start_approach",
        "approach",
    ),
    "approach_complete": (
        "/cylinder_push_execution/arm_contact",
        "/cylinder_push_execution/start_contact",
        "contact",
    ),
    "contact_complete": (
        "/cylinder_push_execution/arm_push",
        "/cylinder_push_execution/start_push",
        "push",
    ),
    "push_complete": (
        "/cylinder_push_execution/arm_return",
        "/cylinder_push_execution/start_return",
        "return",
    ),
    "return_complete": (
        "/cylinder_push_execution/finalize_delivery",
        None,
        "finalize",
    ),
}


RUN_ONCE_SERVICES = {
    "idle": ("/cylinder_push_execution/arm", "arm approach"),
    "armed": ("/cylinder_push_execution/start_approach", "start approach"),
    "approach_complete": (
        "/cylinder_push_execution/arm_contact",
        "arm contact",
    ),
    "contact_armed": (
        "/cylinder_push_execution/start_contact",
        "start contact",
    ),
    "contact_complete": ("/cylinder_push_execution/arm_push", "arm push"),
    "push_armed": ("/cylinder_push_execution/start_push", "start push"),
    "push_complete": ("/cylinder_push_execution/arm_return", "arm return"),
    "return_armed": ("/cylinder_push_execution/start_return", "start return"),
    "return_complete": (
        "/cylinder_push_execution/finalize_delivery",
        "finalize delivery",
    ),
}

RUN_ONCE_ACTIVE_STATES = {
    "preflight",
    "checking_path",
    "approaching",
    "contacting",
    "pushing",
    "releasing",
    "checking_return",
    "returning",
    "finalizing",
}

RUN_ONCE_COMPLETE_STATES = {"collecting_remaining", "task_complete"}
AUTOMATIC_COLLECTION_STATES = {"collecting", "collecting_remaining"}


def collection_snapshot_mode(
    snapshot_ready: bool,
    snapshot_locked: bool,
    validated_count: int,
    elapsed: float,
    timeout: float,
) -> str:
    """Choose whether collection should wait, use a full lock, or fall back.

    A partial fallback contains only targets already marked ``validated`` by
    perception. It does not change the configured inventory, so unseen
    targets remain due in later collection cycles. If the deadline arrives
    before any target is validated, collection continues until the first one
    becomes available rather than entering a terminal failure state.
    """

    if snapshot_ready and snapshot_locked:
        return "full"
    if elapsed < timeout:
        return "waiting"
    if validated_count > 0:
        return "partial"
    return "waiting_for_validated"


def open_inventory_collection_complete(
    open_inventory_mode: bool,
    delivered_count: int,
    snapshot_mode: str,
    snapshot_received: bool,
) -> bool:
    """Finish an open task only after work occurred and the field stayed empty."""

    return (
        open_inventory_mode
        and delivered_count > 0
        and snapshot_mode == "waiting_for_validated"
        and snapshot_received
    )


def advance_services(state: str) -> tuple[str, str | None, str]:
    """Return the gated service sequence for one explicit workflow advance."""

    try:
        return ADVANCE_SERVICES[state]
    except KeyError as error:
        raise ValueError(f"cannot advance execution from state: {state}") from error


def run_once_service(state: str) -> tuple[str, str] | None:
    """Return the next automatic service, or ``None`` while motion is active."""

    if state in RUN_ONCE_SERVICES:
        return RUN_ONCE_SERVICES[state]
    if state in RUN_ONCE_ACTIVE_STATES or state in RUN_ONCE_COMPLETE_STATES:
        return None
    raise ValueError(f"automatic execution cannot continue from state: {state}")


def validate_inventory(colors: list[str], counts: list[int]) -> None:
    """Reject ambiguous or unsafe task inventory parameters."""

    if not colors or len(colors) != len(counts):
        raise ValueError("inventory colors and counts must be non-empty and aligned")
    if any(not color.strip() for color in colors):
        raise ValueError("inventory colors must not be empty")
    if len(set(colors)) != len(colors):
        raise ValueError("inventory colors must be unique")
    if any(count < 0 for count in counts) or sum(counts) < 1:
        raise ValueError("inventory must contain at least one object")


def automatic_cycle_ready(
    state: str,
    snapshot_ready: bool,
    snapshot_locked: bool,
    operation_busy: bool,
) -> bool:
    """Return whether a full-inventory run may begin its next push cycle."""

    return (
        state in AUTOMATIC_COLLECTION_STATES
        and snapshot_ready
        and snapshot_locked
        and not operation_busy
    )


def snapshot_matches_collection(
    expected_colors: list[str] | tuple[str, ...],
    expected_counts: list[int] | tuple[int, ...],
    snapshot_colors: list[str] | tuple[str, ...],
    snapshot_counts: list[int] | tuple[int, ...],
    snapshot_stamp: float,
    collection_started: float,
) -> bool:
    """Accept only a snapshot produced for the current inventory generation."""

    return (
        tuple(str(value) for value in snapshot_colors)
        == tuple(str(value) for value in expected_colors)
        and tuple(int(value) for value in snapshot_counts)
        == tuple(int(value) for value in expected_counts)
        and snapshot_stamp >= collection_started
    )
