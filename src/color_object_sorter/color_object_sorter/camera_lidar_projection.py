"""Lightweight horizontal camera/LiDAR projection and calibration."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import numpy as np


POLYNOMIAL_COEFFICIENT_COUNT = 6


@dataclass(frozen=True)
class ProjectionResult:
    """Projection of one point in the LiDAR frame into normalized image x."""

    normalized_x: float
    bearing: float
    distance: float


@dataclass(frozen=True)
class ProjectionFit:
    """Fitted model and validation metrics."""

    coefficients: tuple[float, ...]
    sample_count: int
    angular_span: float
    distance_span: float
    rmse: float
    maximum_error: float
    leave_one_out_rmse: float


class CameraLidarProjection:
    """Project 2-D LiDAR points with a legacy or calibrated model."""

    def __init__(
        self,
        model: str = "legacy_linear",
        center_normalized_x: float = -0.069,
        radians_per_normalized_x: float = -0.3926990817,
        coefficients: Sequence[float] = (),
    ) -> None:
        if model not in ("legacy_linear", "polynomial_range"):
            raise ValueError(f"unsupported projection model: {model}")
        if not math.isfinite(center_normalized_x):
            raise ValueError("projection center must be finite")
        if not math.isfinite(radians_per_normalized_x) or abs(
            radians_per_normalized_x
        ) < 1e-9:
            raise ValueError("projection scale must be finite and non-zero")
        parsed = tuple(float(value) for value in coefficients)
        if model == "polynomial_range" and len(parsed) != POLYNOMIAL_COEFFICIENT_COUNT:
            raise ValueError(
                "polynomial_range requires exactly six projection coefficients"
            )
        if not all(math.isfinite(value) for value in parsed):
            raise ValueError("projection coefficients must be finite")
        self.model = model
        self.center_normalized_x = center_normalized_x
        self.radians_per_normalized_x = radians_per_normalized_x
        self.coefficients = parsed

    def project(self, x: float, y: float) -> ProjectionResult:
        """Project a point expressed in the LiDAR horizontal frame."""

        distance = math.hypot(x, y)
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError("projection point must have positive finite range")
        bearing = math.atan2(y, x)
        if self.model == "legacy_linear":
            normalized_x = (
                bearing / self.radians_per_normalized_x
                + self.center_normalized_x
            )
        else:
            normalized_x = sum(
                coefficient * feature
                for coefficient, feature in zip(
                    self.coefficients, projection_features(bearing, distance)
                )
            )
        return ProjectionResult(normalized_x, bearing, distance)


def projection_features(bearing: float, distance: float) -> tuple[float, ...]:
    """Return inexpensive angular and inverse-range regression features."""

    if not math.isfinite(bearing) or not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("bearing and distance must be finite with positive distance")
    inverse_range = 1.0 / distance
    return (
        1.0,
        bearing,
        bearing**2,
        bearing**3,
        inverse_range,
        bearing * inverse_range,
    )


def load_projection_samples(path: Path) -> tuple[tuple[float, float, float], ...]:
    """Load ``bearing,distance,normalized_x`` calibration rows."""

    rows = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"bearing", "distance", "normalized_x"}
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError(
                f"CSV is missing columns: {', '.join(sorted(missing))}"
            )
        for line_number, row in enumerate(reader, start=2):
            try:
                values = tuple(float(row[name]) for name in (
                    "bearing",
                    "distance",
                    "normalized_x",
                ))
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"invalid numeric value on CSV line {line_number}"
                ) from error
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite value on CSV line {line_number}")
            if values[1] <= 0.0 or not -1.0 <= values[2] <= 1.0:
                raise ValueError(f"invalid sample on CSV line {line_number}")
            rows.append(values)
    if not rows:
        raise ValueError("CSV contains no projection samples")
    return tuple(rows)


def fit_projection_model(
    samples: Sequence[tuple[float, float, float]],
    minimum_samples: int = 12,
    minimum_angular_span: float = 0.50,
    minimum_distance_span: float = 0.50,
) -> ProjectionFit:
    """Fit and cross-check the six-term lightweight projection model."""

    if len(samples) < minimum_samples:
        raise ValueError(f"at least {minimum_samples} calibration samples are required")
    bearings = np.asarray([item[0] for item in samples], dtype=float)
    distances = np.asarray([item[1] for item in samples], dtype=float)
    observed = np.asarray([item[2] for item in samples], dtype=float)
    angular_span = float(np.ptp(bearings))
    distance_span = float(np.ptp(distances))
    if angular_span < minimum_angular_span:
        raise ValueError("calibration samples do not span enough image angles")
    if distance_span < minimum_distance_span:
        raise ValueError("calibration samples do not span enough target distances")
    design = np.asarray(
        [projection_features(bearing, distance) for bearing, distance in zip(
            bearings,
            distances,
        )],
        dtype=float,
    )
    coefficients, _residuals, rank, _singular = np.linalg.lstsq(
        design,
        observed,
        rcond=None,
    )
    if rank < POLYNOMIAL_COEFFICIENT_COUNT:
        raise ValueError("calibration samples are geometrically degenerate")
    predicted = design @ coefficients
    errors = predicted - observed
    leave_one_out = []
    for index in range(len(samples)):
        reduced_design = np.delete(design, index, axis=0)
        reduced_observed = np.delete(observed, index)
        reduced_coefficients, _residuals, reduced_rank, _singular = np.linalg.lstsq(
            reduced_design,
            reduced_observed,
            rcond=None,
        )
        if reduced_rank < POLYNOMIAL_COEFFICIENT_COUNT:
            raise ValueError("leave-one-out fit is geometrically degenerate")
        leave_one_out.append(
            float(design[index] @ reduced_coefficients - observed[index])
        )
    _validate_monotonic_projection(coefficients, bearings, distances)
    return ProjectionFit(
        coefficients=tuple(float(value) for value in coefficients),
        sample_count=len(samples),
        angular_span=angular_span,
        distance_span=distance_span,
        rmse=float(np.sqrt(np.mean(errors**2))),
        maximum_error=float(np.max(np.abs(errors))),
        leave_one_out_rmse=float(np.sqrt(np.mean(np.asarray(leave_one_out) ** 2))),
    )


def _validate_monotonic_projection(
    coefficients: Sequence[float],
    bearings: np.ndarray,
    distances: np.ndarray,
) -> None:
    derivatives = []
    for bearing in np.linspace(float(bearings.min()), float(bearings.max()), 41):
        for distance in (float(distances.min()), float(distances.max())):
            derivatives.append(
                coefficients[1]
                + 2.0 * coefficients[2] * bearing
                + 3.0 * coefficients[3] * bearing**2
                + coefficients[5] / distance
            )
    if min(derivatives) <= 0.0 <= max(derivatives):
        raise ValueError("fitted projection is not monotonic across the sampled field")


def format_projection_fit(fit: ProjectionFit) -> str:
    """Format diagnostics and a ROS parameter snippet."""

    coefficients = ", ".join(f"{value:.12g}" for value in fit.coefficients)
    return "\n".join(
        (
            "Camera/LiDAR projection fit",
            f"samples: {fit.sample_count}",
            f"angular span: {fit.angular_span:.6f} rad",
            f"distance span: {fit.distance_span:.6f} m",
            f"training RMSE: {fit.rmse:.6f} normalized-x",
            f"maximum error: {fit.maximum_error:.6f} normalized-x",
            f"leave-one-out RMSE: {fit.leave_one_out_rmse:.6f} normalized-x",
            "",
            "Suggested ROS parameters:",
            "projection_model: polynomial_range",
            f"projection_coefficients: [{coefficients}]",
        )
    )
