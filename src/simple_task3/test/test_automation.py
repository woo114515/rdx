"""Test automatic inventory-cycle behavior."""

import pytest

from cylinder_push_planner.inventory_cycle import inventory_after_delivery
from cylinder_push_planner.task_workflow import (
    collection_snapshot_mode,
    open_inventory_collection_complete,
    snapshot_matches_collection,
)


def test_run_all_decrements_exactly_one_object() -> None:
    colors, counts = inventory_after_delivery(
        ("blue", "green", "red"), (2, 2, 2), "green"
    )
    assert colors == ("blue", "green", "red")
    assert counts == (2, 1, 2)


def test_old_snapshot_cannot_start_next_cycle() -> None:
    assert not snapshot_matches_collection(
        ("blue", "green", "red"),
        (2, 1, 2),
        ("blue", "green", "red"),
        (2, 1, 2),
        9.9,
        10.0,
    )


def test_exhausted_color_cannot_be_decremented() -> None:
    with pytest.raises(ValueError):
        inventory_after_delivery(("blue", "green"), (1, 0), "green")


def test_complete_locked_snapshot_is_used_without_waiting() -> None:
    assert collection_snapshot_mode(True, True, 6, 2.0, 15.0) == "full"


def test_validated_subset_is_used_after_collection_timeout() -> None:
    assert collection_snapshot_mode(False, False, 4, 15.0, 15.0) == "partial"


def test_partial_snapshot_waits_until_collection_timeout() -> None:
    assert collection_snapshot_mode(False, False, 4, 14.9, 15.0) == "waiting"


def test_timeout_without_validated_target_keeps_collecting() -> None:
    assert (
        collection_snapshot_mode(False, False, 0, 15.0, 15.0)
        == "waiting_for_validated"
    )


def test_open_inventory_finishes_after_an_empty_post_delivery_window() -> None:
    assert open_inventory_collection_complete(
        True, 1, "waiting_for_validated", True
    )
    assert not open_inventory_collection_complete(
        True, 0, "waiting_for_validated", True
    )
    assert not open_inventory_collection_complete(True, 1, "partial", True)
    assert not open_inventory_collection_complete(
        False, 1, "waiting_for_validated", True
    )
    assert not open_inventory_collection_complete(
        True, 1, "waiting_for_validated", False
    )
