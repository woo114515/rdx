"""Test automatic inventory-cycle behavior."""

import pytest

from cylinder_push_planner.inventory_cycle import inventory_after_delivery
from cylinder_push_planner.task_workflow import snapshot_matches_collection


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
