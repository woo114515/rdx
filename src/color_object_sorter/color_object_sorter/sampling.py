"""Pure helpers for collecting labelled HSV calibration samples."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class HsvPixel:
    """One pixel sampled from a labelled image patch."""

    offset_x: int
    offset_y: int
    hue: int
    saturation: int
    value: int


def extract_hsv_patch(
    image: np.ndarray,
    center_x: int,
    center_y: int,
    radius: int,
) -> tuple[HsvPixel, ...]:
    """Return all HSV pixels in a clipped square around an image position."""

    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image must be a BGR image")
    if radius < 0:
        raise ValueError("radius must be non-negative")
    height, width = image.shape[:2]
    if not 0 <= center_x < width or not 0 <= center_y < height:
        raise ValueError("sample center must be inside the image")

    left = max(0, center_x - radius)
    right = min(width, center_x + radius + 1)
    top = max(0, center_y - radius)
    bottom = min(height, center_y + radius + 1)
    hsv = cv2.cvtColor(image[top:bottom, left:right], cv2.COLOR_BGR2HSV)
    pixels = []
    for patch_y in range(hsv.shape[0]):
        for patch_x in range(hsv.shape[1]):
            hue, saturation, value = hsv[patch_y, patch_x]
            pixels.append(
                HsvPixel(
                    offset_x=left + patch_x - center_x,
                    offset_y=top + patch_y - center_y,
                    hue=int(hue),
                    saturation=int(saturation),
                    value=int(value),
                )
            )
    return tuple(pixels)
