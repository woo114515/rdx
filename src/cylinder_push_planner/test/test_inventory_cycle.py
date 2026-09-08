import pytest

from cylinder_push_planner.inventory_cycle import inventory_after_delivery


def test_delivery_decrements_only_selected_color() -> None:
    colors, counts = inventory_after_delivery(
        ["blue", "green", "red"], [2, 2, 2], "green"
    )

    assert colors == ("blue", "green", "red")
    assert counts == (2, 1, 2)


def test_delivery_can_exhaust_a_color_without_removing_schema() -> None:
    colors, counts = inventory_after_delivery(["blue", "red"], [1, 2], "blue")

    assert colors == ("blue", "red")
    assert counts == (0, 2)


def test_last_delivery_produces_task_complete_inventory() -> None:
    colors, counts = inventory_after_delivery(["red"], [1], "red")

    assert colors == ("red",)
    assert counts == (0,)


@pytest.mark.parametrize(
    ("colors", "counts", "delivered"),
    [
        (["blue"], [], "blue"),
        (["blue"], [1], "red"),
        (["blue"], [0], "blue"),
        (["blue"], [-1], "blue"),
    ],
)
def test_invalid_inventory_transition_is_rejected(colors, counts, delivered) -> None:
    with pytest.raises(ValueError):
        inventory_after_delivery(colors, counts, delivered)
