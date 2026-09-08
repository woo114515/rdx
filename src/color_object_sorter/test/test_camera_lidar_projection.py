from pathlib import Path

import pytest

from color_object_sorter.camera_lidar_projection import (
    CameraLidarProjection,
    fit_projection_model,
    load_projection_samples,
    projection_features,
)


def test_legacy_projection_preserves_two_point_formula() -> None:
    model = CameraLidarProjection(
        center_normalized_x=-0.069,
        radians_per_normalized_x=-0.4,
    )
    projected = model.project(1.0, 0.0)
    assert projected.normalized_x == pytest.approx(-0.069)
    assert projected.distance == pytest.approx(1.0)


def test_polynomial_projection_includes_range_terms() -> None:
    coefficients = (-0.05, -2.2, 0.1, -0.2, 0.04, 0.05)
    model = CameraLidarProjection(
        model="polynomial_range",
        coefficients=coefficients,
    )
    result = model.project(1.2, 0.3)
    expected = sum(
        coefficient * feature
        for coefficient, feature in zip(
            coefficients,
            projection_features(result.bearing, result.distance),
        )
    )
    assert result.normalized_x == pytest.approx(expected)


def test_fit_recovers_synthetic_monotonic_projection() -> None:
    coefficients = (-0.05, -2.2, 0.1, -0.2, 0.04, 0.05)
    samples = []
    for bearing in (-0.32, -0.16, 0.0, 0.16, 0.32):
        for distance in (0.8, 1.6, 2.5):
            normalized_x = sum(
                coefficient * feature
                for coefficient, feature in zip(
                    coefficients,
                    projection_features(bearing, distance),
                )
            )
            samples.append((bearing, distance, normalized_x))
    fit = fit_projection_model(samples)
    assert fit.coefficients == pytest.approx(coefficients, abs=1e-9)
    assert fit.rmse < 1e-10
    assert fit.leave_one_out_rmse < 1e-9


def test_fit_rejects_inadequate_angular_coverage() -> None:
    samples = [(0.01 * index, 1.0 + 0.1 * index, 0.0) for index in range(12)]
    with pytest.raises(ValueError, match="angles"):
        fit_projection_model(samples)


def test_projection_csv_loader_validates_required_columns(tmp_path: Path) -> None:
    path = tmp_path / "bad.csv"
    path.write_text("bearing,distance\n0.0,1.0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="normalized_x"):
        load_projection_samples(path)
