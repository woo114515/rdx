"""Pure inventory transitions after one cylinder has been delivered."""

from __future__ import annotations

from typing import Sequence


def inventory_after_delivery(
    colors: Sequence[str],
    counts: Sequence[int],
    delivered_color: str,
) -> tuple[tuple[str, ...], tuple[int, ...]]:
    """Return the remaining inventory after exactly one delivered object."""

    names = tuple(str(color).strip() for color in colors)
    values = tuple(int(count) for count in counts)
    delivered = str(delivered_color).strip()
    if len(names) != len(values):
        raise ValueError("inventory colors and counts must have equal lengths")
    if not delivered or delivered not in names:
        raise ValueError("delivered color is absent from inventory")
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError("inventory color names must be non-empty and unique")
    if any(count < 0 for count in values):
        raise ValueError("inventory counts must be non-negative")
    index = names.index(delivered)
    if values[index] == 0:
        raise ValueError("delivered color is already exhausted")
    remaining = list(values)
    remaining[index] -= 1
    return names, tuple(remaining)
