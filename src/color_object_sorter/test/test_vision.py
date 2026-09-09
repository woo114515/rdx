from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from color_object_sorter.vision import ColorObjectDetector, HsvRange


def make_detector() -> ColorObjectDetector:
    return ColorObjectDetector(
        {"green": (HsvRange((35, 80, 60), (85, 255, 255)),)},
        roi_top_ratio=0.0,
        minimum_area_ratio=0.005,
    )


def make_multi_detector() -> ColorObjectDetector:
    return ColorObjectDetector(
        {
            "blue": (HsvRange((90, 80, 50), (135, 255, 255)),),
            "green": (HsvRange((35, 80, 60), (85, 255, 255)),),
            "red": (
                HsvRange((0, 50, 80), (12, 255, 255)),
                HsvRange((165, 50, 80), (179, 255, 255)),
            ),
        },
        roi_top_ratio=0.0,
        minimum_area_ratio=0.005,
    )


def test_detects_two_objects_of_same_color() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 60), (70, 180), (0, 255, 0), -1)
    cv2.rectangle(image, (210, 50), (250, 170), (0, 255, 0), -1)
    result = make_detector().detect(image)
    assert len(result.detections) == 2
    assert {item.color for item in result.detections} == {"green"}


def test_rejects_wide_color_region() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 100), (300, 150), (0, 255, 0), -1)
    assert make_detector().detect(image).detections == ()


def test_hsv_range_rejects_invalid_values() -> None:
    try:
        HsvRange((180, 0, 0), (180, 255, 255))
    except ValueError:
        return
    raise AssertionError("invalid hue must be rejected")


def test_detects_blue_green_and_red() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 40), (70, 160), (255, 0, 0), -1)  # blue
    cv2.rectangle(image, (130, 40), (180, 160), (0, 255, 0), -1)  # green
    cv2.rectangle(image, (230, 40), (280, 160), (0, 0, 255), -1)  # red
    result = make_multi_detector().detect(image)
    assert {item.color for item in result.detections} == {"blue", "green", "red"}


def test_red_detected_in_both_hue_ranges() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 40), (70, 160), (0, 0, 255), -1)  # hue ~0
    cv2.rectangle(image, (230, 40), (280, 160), (8, 0, 120), -1)  # hue ~178
    result = make_multi_detector().detect(image)
    red = [item for item in result.detections if item.color == "red"]
    assert len(red) == 2


def test_active_red_config_appends_dark_ranges_without_replacing_old_ones() -> None:
    config = Path(__file__).parents[1] / "config" / "perception.yaml"
    parameters = yaml.safe_load(config.read_text(encoding="utf-8"))[
        "color_object_detector"
    ]["ros__parameters"]

    assert parameters["hsv.red.range_count"] == 4
    assert parameters["hsv.red.lower_1"] == [0, 155, 90]
    assert parameters["hsv.red.upper_1"] == [4, 255, 195]
    assert parameters["hsv.red.lower_2"] == [170, 155, 90]
    assert parameters["hsv.red.upper_2"] == [179, 255, 195]
    assert parameters["hsv.red.lower_3"] == [0, 219, 37]
    assert parameters["hsv.red.upper_3"] == [8, 255, 109]
    assert parameters["hsv.red.lower_4"] == [177, 219, 37]
    assert parameters["hsv.red.upper_4"] == [179, 255, 109]


def test_filters_small_area_noise() -> None:
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (40, 40), (50, 50), (0, 255, 0), -1)
    cv2.rectangle(image, (200, 200), (210, 210), (0, 255, 0), -1)
    assert make_detector().detect(image).detections == ()


def test_rejects_invalid_area_ratio_bounds() -> None:
    with pytest.raises(ValueError):
        ColorObjectDetector(
            {"green": (HsvRange((35, 80, 60), (85, 255, 255)),)},
            minimum_area_ratio=0.5,
            maximum_area_ratio=0.1,
        )


def test_rejects_invalid_aspect_ratio_bounds() -> None:
    with pytest.raises(ValueError):
        ColorObjectDetector(
            {"green": (HsvRange((35, 80, 60), (85, 255, 255)),)},
            minimum_aspect_ratio=2.0,
            maximum_aspect_ratio=1.0,
        )


def test_rejects_out_of_range_extent_and_solidity() -> None:
    with pytest.raises(ValueError):
        ColorObjectDetector(
            {"green": (HsvRange((35, 80, 60), (85, 255, 255)),)},
            minimum_extent=1.5,
        )
    with pytest.raises(ValueError):
        ColorObjectDetector(
            {"green": (HsvRange((35, 80, 60), (85, 255, 255)),)},
            minimum_solidity=-0.1,
        )
