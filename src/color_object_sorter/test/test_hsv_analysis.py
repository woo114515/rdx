import csv

import pytest

from color_object_sorter.hsv_analysis import analyse_samples, load_samples


def test_loads_sampler_csv_and_counts_clicks(tmp_path) -> None:
    path = tmp_path / "samples.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=("sample_id", "label", "h", "s", "v")
        )
        writer.writeheader()
        writer.writerows(
            (
                {"sample_id": 1, "label": "blue", "h": 90, "s": 200, "v": 150},
                {"sample_id": 1, "label": "blue", "h": 91, "s": 201, "v": 151},
                {"sample_id": 2, "label": "blue", "h": 92, "s": 202, "v": 152},
            )
        )

    result = analyse_samples(
        load_samples(path), hue_margin=0, saturation_margin=0, value_margin=0
    )

    assert result[0].sample_count == 2
    assert result[0].pixel_count == 3


def test_splits_red_hue_across_opencv_boundary() -> None:
    samples = {
        "red": [
            (1, 178, 220, 180),
            (1, 179, 221, 181),
            (2, 0, 222, 182),
            (2, 1, 223, 183),
        ]
    }

    result = analyse_samples(samples, hue_margin=1)[0]

    assert len(result.ranges) == 2
    assert result.ranges[0].lower[0] == 0
    assert result.ranges[1].upper[0] == 179


def test_reports_background_overlap() -> None:
    samples = {
        "green": [(1, 65, 100, 120), (2, 66, 110, 130)],
        "background": [(1, 65, 105, 125), (2, 20, 20, 20)],
    }

    result = analyse_samples(
        samples, hue_margin=0, saturation_margin=0, value_margin=0
    )[0]

    assert result.background_overlap == pytest.approx(0.5)


def test_black_uses_full_hue_and_saturation_ranges() -> None:
    samples = {
        "black": [(1, 0, 0, 15), (2, 90, 255, 25), (3, 179, 80, 35)],
        "background": [(4, 60, 20, 20), (5, 60, 20, 180)],
    }

    result = analyse_samples(
        samples,
        labels=("black",),
        hue_margin=0,
        saturation_margin=0,
        value_margin=0,
    )[0]

    assert result.ranges[0].lower == (0, 0, 0)
    assert result.ranges[0].upper[:2] == (179, 255)
    assert result.background_overlap == pytest.approx(0.5)


def test_rejects_missing_columns(tmp_path) -> None:
    path = tmp_path / "bad.csv"
    path.write_text("label,h,s\nblue,90,200\n", encoding="utf-8")

    with pytest.raises(ValueError, match="missing columns"):
        load_samples(path)
