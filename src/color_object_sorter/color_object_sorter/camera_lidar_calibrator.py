"""Command-line fitter for captured camera/LiDAR correspondences."""

from __future__ import annotations

import argparse
from pathlib import Path

from .camera_lidar_projection import (
    fit_projection_model,
    format_projection_fit,
    load_projection_samples,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--minimum-samples", type=int, default=12)
    parser.add_argument("--minimum-angular-span", type=float, default=0.50)
    parser.add_argument("--minimum-distance-span", type=float, default=0.50)
    arguments = parser.parse_args()
    fit = fit_projection_model(
        load_projection_samples(arguments.csv_file),
        minimum_samples=arguments.minimum_samples,
        minimum_angular_span=arguments.minimum_angular_span,
        minimum_distance_span=arguments.minimum_distance_span,
    )
    print(format_projection_fit(fit))


if __name__ == "__main__":
    main()
