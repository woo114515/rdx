"""Validation for configurable object/color inventory constraints."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class ObjectInventory:
    """Expected object counts by arbitrary color name."""

    counts: tuple[tuple[str, int], ...]

    @classmethod
    def from_lists(
        cls,
        colors: Sequence[str],
        counts: Sequence[int],
    ) -> "ObjectInventory":
        names = tuple(str(color).strip() for color in colors)
        values = tuple(int(count) for count in counts)
        if len(names) != len(values):
            raise ValueError("inventory colors and counts must have equal lengths")
        if any(not name for name in names):
            raise ValueError("inventory color names cannot be empty")
        if len(names) != len(set(names)):
            raise ValueError("inventory color names must be unique")
        if any(count < 1 for count in values):
            raise ValueError("inventory counts must be positive")
        return cls(tuple(zip(names, values)))

    @property
    def colors(self) -> tuple[str, ...]:
        return tuple(color for color, _count in self.counts)

    @property
    def color_counts(self) -> tuple[int, ...]:
        return tuple(count for _color, count in self.counts)

    @property
    def total(self) -> int:
        return sum(self.color_counts)
