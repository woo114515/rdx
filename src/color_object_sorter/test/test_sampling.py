import cv2
import numpy as np
import pytest

from color_object_sorter.sampling import extract_hsv_patch


def test_extracts_centered_patch() -> None:
    image = np.zeros((5, 5, 3), dtype=np.uint8)
    image[:] = (0, 255, 0)
    pixels = extract_hsv_patch(image, 2, 2, 1)
    assert len(pixels) == 9
    assert {(item.hue, item.saturation, item.value) for item in pixels} == {
        (60, 255, 255)
    }


def test_clips_patch_at_image_edge() -> None:
    image = np.zeros((5, 5, 3), dtype=np.uint8)
    pixels = extract_hsv_patch(image, 0, 0, 2)
    assert len(pixels) == 9
    assert min(item.offset_x for item in pixels) == 0
    assert max(item.offset_x for item in pixels) == 2


def test_preserves_opencv_hue_range() -> None:
    hsv = np.full((1, 1, 3), (178, 200, 150), dtype=np.uint8)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    pixel = extract_hsv_patch(image, 0, 0, 0)[0]
    assert pixel.hue == 178
    assert pixel.value == 150


def test_rejects_invalid_sampling_request() -> None:
    image = np.zeros((5, 5, 3), dtype=np.uint8)
    with pytest.raises(ValueError):
        extract_hsv_patch(image, -1, 0, 1)
    with pytest.raises(ValueError):
        extract_hsv_patch(image, 0, 0, -1)
    with pytest.raises(ValueError):
        extract_hsv_patch(np.zeros((5, 5), dtype=np.uint8), 0, 0, 1)
