"""Pure image decoding and sizing helpers for the debug viewer."""

from __future__ import annotations

import cv2
import numpy as np


def decode_jpeg(data: bytes) -> np.ndarray | None:
    """Decode JPEG bytes, returning ``None`` for invalid input."""
    if not data:
        return None
    return cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)


def fit_image(image: np.ndarray, maximum_width: int, maximum_height: int) -> np.ndarray:
    """Scale an image down to fit requested bounds without enlarging it."""
    if maximum_width < 1 or maximum_height < 1:
        raise ValueError("display bounds must be positive")
    height, width = image.shape[:2]
    scale = min(1.0, maximum_width / width, maximum_height / height)
    if scale == 1.0:
        return image
    return cv2.resize(
        image,
        (max(1, round(width * scale)), max(1, round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )
