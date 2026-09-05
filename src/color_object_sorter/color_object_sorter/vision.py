"""Hardware-independent multi-object HSV segmentation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import cv2
import numpy as np


@dataclass(frozen=True)
class HsvRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]

    def __post_init__(self) -> None:
        if len(self.lower) != 3 or len(self.upper) != 3:
            raise ValueError("HSV bounds must contain three values")
        limits = (179, 255, 255)
        for lower, upper, limit in zip(self.lower, self.upper, limits):
            if not 0 <= lower <= upper <= limit:
                raise ValueError("invalid HSV bounds")


@dataclass(frozen=True)
class Detection:
    color: str
    x: int
    y: int
    width: int
    height: int
    center_x: float
    center_y: float
    normalized_x: float
    normalized_y: float
    area_ratio: float
    confidence: float
    shape: str = "cylinder"


@dataclass(frozen=True)
class VisionResult:
    detections: tuple[Detection, ...]
    debug_image: np.ndarray


class ColorObjectDetector:
    """Return every valid color object, rather than one contour per color."""

    def __init__(
        self,
        ranges: Mapping[str, Sequence[HsvRange]],
        *,
        minimum_area_ratio: float = 0.001,
        maximum_area_ratio: float = 0.35,
        minimum_aspect_ratio: float = 0.15,
        maximum_aspect_ratio: float = 1.10,
        minimum_extent: float = 0.40,
        minimum_solidity: float = 0.75,
        roi_top_ratio: float = 0.10,
        morphology_kernel_size: int = 5,
    ) -> None:
        if not ranges or any(not items for items in ranges.values()):
            raise ValueError("every color must have at least one HSV range")
        if not 0.0 <= minimum_area_ratio < maximum_area_ratio <= 1.0:
            raise ValueError("area ratio bounds must satisfy 0 <= min < max <= 1")
        if not 0.0 <= minimum_aspect_ratio <= maximum_aspect_ratio:
            raise ValueError("aspect ratio bounds must satisfy 0 <= min <= max")
        if not 0.0 <= minimum_extent <= 1.0:
            raise ValueError("minimum_extent must be in [0, 1]")
        if not 0.0 <= minimum_solidity <= 1.0:
            raise ValueError("minimum_solidity must be in [0, 1]")
        if not 0.0 <= roi_top_ratio < 1.0:
            raise ValueError("roi_top_ratio must be in [0, 1)")
        if morphology_kernel_size < 1 or morphology_kernel_size % 2 == 0:
            raise ValueError("morphology kernel must be a positive odd number")
        self.ranges = {name: tuple(items) for name, items in ranges.items()}
        self.minimum_area_ratio = minimum_area_ratio
        self.maximum_area_ratio = maximum_area_ratio
        self.minimum_aspect_ratio = minimum_aspect_ratio
        self.maximum_aspect_ratio = maximum_aspect_ratio
        self.minimum_extent = minimum_extent
        self.minimum_solidity = minimum_solidity
        self.roi_top_ratio = roi_top_ratio
        self.kernel = np.ones(
            (morphology_kernel_size, morphology_kernel_size), dtype=np.uint8
        )

    def detect(self, image: np.ndarray) -> VisionResult:
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must be a BGR image")
        image_height, image_width = image.shape[:2]
        image_area = float(image_width * image_height)
        roi_top = int(image_height * self.roi_top_ratio)
        hsv = cv2.cvtColor(image[roi_top:, :], cv2.COLOR_BGR2HSV)
        debug = image.copy()
        detections: list[Detection] = []

        for color, ranges in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for item in ranges:
                mask = cv2.bitwise_or(
                    mask,
                    cv2.inRange(
                        hsv,
                        np.asarray(item.lower, dtype=np.uint8),
                        np.asarray(item.upper, dtype=np.uint8),
                    ),
                )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                detection = self._from_contour(
                    color, contour, image_width, image_height, image_area, roi_top
                )
                if detection is None:
                    continue
                detections.append(detection)
                self._draw(debug, detection)

        detections.sort(key=lambda item: (-item.area_ratio, item.color, item.x))
        return VisionResult(tuple(detections), debug)

    def _from_contour(
        self,
        color: str,
        contour: np.ndarray,
        image_width: int,
        image_height: int,
        image_area: float,
        roi_top: int,
    ) -> Detection | None:
        contour_area = float(cv2.contourArea(contour))
        area_ratio = contour_area / image_area
        if not self.minimum_area_ratio <= area_ratio <= self.maximum_area_ratio:
            return None
        x, roi_y, width, height = cv2.boundingRect(contour)
        if width <= 0 or height <= 0:
            return None
        aspect_ratio = width / height
        extent = contour_area / float(width * height)
        hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = contour_area / hull_area if hull_area > 0.0 else 0.0
        if not (
            self.minimum_aspect_ratio <= aspect_ratio <= self.maximum_aspect_ratio
            and extent >= self.minimum_extent
            and solidity >= self.minimum_solidity
        ):
            return None
        y = roi_y + roi_top
        center_x = x + width / 2.0
        center_y = y + height / 2.0
        confidence = min(1.0, 0.5 * extent + 0.5 * solidity)
        return Detection(
            color=color,
            x=x,
            y=y,
            width=width,
            height=height,
            center_x=center_x,
            center_y=center_y,
            normalized_x=(center_x - image_width / 2.0) / (image_width / 2.0),
            normalized_y=(center_y - image_height / 2.0) / (image_height / 2.0),
            area_ratio=area_ratio,
            confidence=confidence,
        )

    @staticmethod
    def _draw(image: np.ndarray, detection: Detection) -> None:
        start = (detection.x, detection.y)
        end = (detection.x + detection.width, detection.y + detection.height)
        cv2.rectangle(image, start, end, (255, 255, 255), 2)
        cv2.putText(
            image,
            f"{detection.color} {detection.confidence:.2f}",
            (detection.x, max(18, detection.y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
