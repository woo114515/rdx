"""Command-line entry point for repeatable HSV CSV analysis."""

from __future__ import annotations

import argparse
from pathlib import Path

from .hsv_analysis import analyse_samples, format_report, load_samples


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--labels", nargs="+", help="target labels to analyse")
    parser.add_argument("--low-quantile", type=float, default=0.02)
    parser.add_argument("--high-quantile", type=float, default=0.98)
    parser.add_argument("--hue-margin", type=int, default=2)
    parser.add_argument("--saturation-margin", type=int, default=10)
    parser.add_argument("--value-margin", type=int, default=10)
    arguments = parser.parse_args()

    analyses = analyse_samples(
        load_samples(arguments.csv_file),
        labels=arguments.labels,
        low_quantile=arguments.low_quantile,
        high_quantile=arguments.high_quantile,
        hue_margin=arguments.hue_margin,
        saturation_margin=arguments.saturation_margin,
        value_margin=arguments.value_margin,
    )
    print(format_report(analyses))


if __name__ == "__main__":
    main()
