"""Analyse labelled HSV samples and propose OpenCV HSV thresholds."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


IGNORED_LABELS = frozenset({"background", "yellow_distractor"})


@dataclass(frozen=True)
class HsvRange:
    """One inclusive OpenCV HSV range."""

    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


@dataclass(frozen=True)
class LabelAnalysis:
    """Threshold proposal and diagnostics for one target label."""

    label: str
    sample_count: int
    pixel_count: int
    ranges: tuple[HsvRange, ...]
    background_overlap: float


def load_samples(path: Path) -> dict[str, list[tuple[int, int, int, int]]]:
    """Load rows as ``label -> (sample_id, h, s, v)``."""

    result: dict[str, list[tuple[int, int, int, int]]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"sample_id", "label", "h", "s", "v"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(f"CSV is missing columns: {', '.join(sorted(missing))}")
        for line_number, row in enumerate(reader, start=2):
            label = row["label"].strip()
            if not label:
                raise ValueError(f"empty label on CSV line {line_number}")
            try:
                values = (
                    int(row["sample_id"]),
                    int(row["h"]),
                    int(row["s"]),
                    int(row["v"]),
                )
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid numeric value on CSV line {line_number}") from error
            if not (0 <= values[1] <= 179 and 0 <= values[2] <= 255 and 0 <= values[3] <= 255):
                raise ValueError(f"HSV value outside OpenCV range on CSV line {line_number}")
            result.setdefault(label, []).append(values)
    if not result:
        raise ValueError("CSV contains no samples")
    return result


def _bounded_quantiles(
    values: np.ndarray,
    low_quantile: float,
    high_quantile: float,
    margin: int,
    maximum: int,
) -> tuple[int, int]:
    low = int(np.floor(np.quantile(values, low_quantile))) - margin
    high = int(np.ceil(np.quantile(values, high_quantile))) + margin
    return max(0, low), min(maximum, high)


def _hue_ranges(
    hues: np.ndarray,
    low_quantile: float,
    high_quantile: float,
    margin: int,
) -> tuple[tuple[int, int], ...]:
    """Find a robust circular hue interval, split at 0 when necessary."""

    angles = hues.astype(float) * (2.0 * np.pi / 180.0)
    center = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean())
    center_hue = (center * 180.0 / (2.0 * np.pi)) % 180.0
    offsets = ((hues.astype(float) - center_hue + 90.0) % 180.0) - 90.0
    low = float(np.quantile(offsets, low_quantile)) - margin
    high = float(np.quantile(offsets, high_quantile)) + margin
    if high - low >= 179.0:
        return ((0, 179),)
    start = center_hue + low
    end = center_hue + high
    while start < 0.0:
        start += 180.0
        end += 180.0
    while start >= 180.0:
        start -= 180.0
        end -= 180.0
    start_i = max(0, int(np.floor(start)))
    if end <= 179.0:
        return ((start_i, min(179, int(np.ceil(end)))),)
    return ((0, min(179, int(np.ceil(end - 180.0)))), (start_i, 179))


def _matches(pixel: tuple[int, int, int, int], ranges: Iterable[HsvRange]) -> bool:
    _, hue, saturation, value = pixel
    return any(
        item.lower[0] <= hue <= item.upper[0]
        and item.lower[1] <= saturation <= item.upper[1]
        and item.lower[2] <= value <= item.upper[2]
        for item in ranges
    )


def analyse_samples(
    samples: dict[str, list[tuple[int, int, int, int]]],
    labels: Iterable[str] | None = None,
    low_quantile: float = 0.02,
    high_quantile: float = 0.98,
    hue_margin: int = 2,
    saturation_margin: int = 10,
    value_margin: int = 10,
) -> tuple[LabelAnalysis, ...]:
    """Return robust threshold proposals and background false-positive rates."""

    if not 0.0 <= low_quantile < high_quantile <= 1.0:
        raise ValueError("quantiles must satisfy 0 <= low < high <= 1")
    selected = sorted(labels if labels is not None else set(samples) - IGNORED_LABELS)
    background = samples.get("background", [])
    analyses = []
    for label in selected:
        rows = samples.get(label, [])
        if not rows:
            raise ValueError(f"no samples for label {label!r}")
        values = np.asarray([(h, s, v) for _, h, s, v in rows], dtype=float)
        saturation = _bounded_quantiles(
            values[:, 1], low_quantile, high_quantile, saturation_margin, 255
        )
        value = _bounded_quantiles(
            values[:, 2], low_quantile, high_quantile, value_margin, 255
        )
        if label == "black":
            # Hue and saturation become unstable when brightness approaches
            # zero. Black is therefore a full-H/S, low-value classification.
            ranges = (
                HsvRange(lower=(0, 0, 0), upper=(179, 255, value[1])),
            )
        else:
            ranges = tuple(
                HsvRange(
                    lower=(hue_low, saturation[0], value[0]),
                    upper=(hue_high, saturation[1], value[1]),
                )
                for hue_low, hue_high in _hue_ranges(
                    values[:, 0], low_quantile, high_quantile, hue_margin
                )
            )
        overlap = (
            sum(_matches(pixel, ranges) for pixel in background) / len(background)
            if background
            else 0.0
        )
        analyses.append(
            LabelAnalysis(
                label=label,
                sample_count=len({row[0] for row in rows}),
                pixel_count=len(rows),
                ranges=ranges,
                background_overlap=overlap,
            )
        )
    return tuple(analyses)


def format_report(analyses: Iterable[LabelAnalysis]) -> str:
    """Format a human-readable report followed by a ROS parameter snippet."""

    items = tuple(analyses)
    lines = ["HSV sample analysis", ""]
    for item in items:
        lines.append(
            f"{item.label}: {item.sample_count} clicks, {item.pixel_count} pixels, "
            f"background overlap={item.background_overlap:.2%}"
        )
    lines.extend(("", "Suggested ROS parameters:"))
    for item in items:
        lines.append(f"hsv.{item.label}.range_count: {len(item.ranges)}")
        for index, hsv_range in enumerate(item.ranges, start=1):
            lines.append(f"hsv.{item.label}.lower_{index}: {list(hsv_range.lower)}")
            lines.append(f"hsv.{item.label}.upper_{index}: {list(hsv_range.upper)}")
    return "\n".join(lines)
