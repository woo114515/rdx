"""OpenCV color-segmentation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import cv2
import numpy as np

from .behavior import Detection
from .shape import matches_bottle_shape, matches_target_shape


@dataclass(frozen=True)
class HsvRange:
    lower: tuple[int, int, int]
    upper: tuple[int, int, int]


@dataclass(frozen=True)
class VisionResult:
    detections: tuple[Detection, ...]
    debug_image: np.ndarray


class ColorDetector:
    """Find the largest connected region for each configured color."""

    def __init__(
        self,
        ranges: Mapping[str, Sequence[HsvRange]],
        minimum_area_ratio: float = 0.001,
        maximum_area_ratio: float = 0.35,
        roi_top_ratio: float = 0.25,
        minimum_width_height_ratio: float = 0.25,
        maximum_width_height_ratio: float = 0.90,
        minimum_extent: float = 0.55,
        minimum_solidity: float = 0.88,
        bottle_detection_enabled: bool = True,
        bottle_minimum_width_height_ratio: float = 0.15,
        bottle_maximum_width_height_ratio: float = 0.65,
        bottle_minimum_extent: float = 0.35,
        bottle_minimum_solidity: float = 0.75,
        bottle_maximum_neck_body_width_ratio: float = 0.78,
    ) -> None:
        self.ranges = {color: tuple(items) for color, items in ranges.items()}
        self.minimum_area_ratio = minimum_area_ratio
        self.maximum_area_ratio = maximum_area_ratio
        self.roi_top_ratio = roi_top_ratio
        self.minimum_width_height_ratio = minimum_width_height_ratio
        self.maximum_width_height_ratio = maximum_width_height_ratio
        self.minimum_extent = minimum_extent
        self.minimum_solidity = minimum_solidity
        self.bottle_detection_enabled = bottle_detection_enabled
        self.bottle_minimum_width_height_ratio = (
            bottle_minimum_width_height_ratio
        )
        self.bottle_maximum_width_height_ratio = (
            bottle_maximum_width_height_ratio
        )
        self.bottle_minimum_extent = bottle_minimum_extent
        self.bottle_minimum_solidity = bottle_minimum_solidity
        self.bottle_maximum_neck_body_width_ratio = (
            bottle_maximum_neck_body_width_ratio
        )

    def detect(self, image: np.ndarray) -> VisionResult:
        if image is None or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("image must be a BGR image with three channels")
        height, width = image.shape[:2]
        roi_top = int(height * self.roi_top_ratio)
        roi = image[roi_top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        debug = image.copy()
        detections: list[Detection] = []
        kernel = np.ones((5, 5), dtype=np.uint8)
        image_area = float(width * height)

        for color, color_ranges in self.ranges.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for hsv_range in color_ranges:
                lower = np.array(hsv_range.lower, dtype=np.uint8)
                upper = np.array(hsv_range.upper, dtype=np.uint8)
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            selected: tuple[
                np.ndarray,
                float,
                tuple[int, int, int, int],
                str,
            ] | None = None
            for contour in sorted(
                contours,
                key=cv2.contourArea,
                reverse=True,
            ):
                contour_area = cv2.contourArea(contour)
                area_ratio = contour_area / image_area
                if area_ratio < self.minimum_area_ratio:
                    break
                bounds = cv2.boundingRect(contour)
                _, _, box_width, box_height = bounds
                hull_area = cv2.contourArea(cv2.convexHull(contour))
                neck_body_ratio = self._neck_body_width_ratio(contour, bounds)
                is_bottle = (
                    self.bottle_detection_enabled
                    and neck_body_ratio is not None
                    and matches_bottle_shape(
                        contour_area=contour_area,
                        hull_area=hull_area,
                        area_ratio=area_ratio,
                        box_width=box_width,
                        box_height=box_height,
                        neck_body_width_ratio=neck_body_ratio,
                        minimum_width_height_ratio=(
                            self.bottle_minimum_width_height_ratio
                        ),
                        maximum_width_height_ratio=(
                            self.bottle_maximum_width_height_ratio
                        ),
                        minimum_extent=self.bottle_minimum_extent,
                        minimum_solidity=self.bottle_minimum_solidity,
                        maximum_neck_body_width_ratio=(
                            self.bottle_maximum_neck_body_width_ratio
                        ),
                        minimum_area_ratio=self.minimum_area_ratio,
                        maximum_area_ratio=self.maximum_area_ratio,
                    )
                )
                is_cuboid = matches_target_shape(
                    contour_area=contour_area,
                    hull_area=hull_area,
                    area_ratio=area_ratio,
                    box_width=box_width,
                    box_height=box_height,
                    minimum_width_height_ratio=(
                        self.minimum_width_height_ratio
                    ),
                    maximum_width_height_ratio=(
                        self.maximum_width_height_ratio
                    ),
                    minimum_extent=self.minimum_extent,
                    minimum_solidity=self.minimum_solidity,
                    minimum_area_ratio=self.minimum_area_ratio,
                    maximum_area_ratio=self.maximum_area_ratio,
                )
                if is_bottle or is_cuboid:
                    shape = "bottle" if is_bottle else "cuboid"
                    selected = (contour, area_ratio, bounds, shape)
                    break
            if selected is None:
                continue
            _, area_ratio, bounds, shape = selected
            x, y, box_width, box_height = bounds
            center_x = x + box_width / 2.0
            x_error = (center_x - width / 2.0) / (width / 2.0)
            detections.append(
                Detection(
                    color=color,
                    x_error=float(x_error),
                    area_ratio=float(area_ratio),
                    shape=shape,
                )
            )
            cv2.rectangle(
                debug,
                (x, y + roi_top),
                (x + box_width, y + roi_top + box_height),
                (255, 255, 255),
                2,
            )
            cv2.putText(
                debug,
                f"{color}/{shape} {area_ratio:.3f} "
                f"r={box_width / box_height:.2f}",
                (x, max(20, y + roi_top - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return VisionResult(tuple(detections), debug)

    @staticmethod
    def _neck_body_width_ratio(
        contour: np.ndarray,
        bounds: tuple[int, int, int, int],
    ) -> float | None:
        """Measure upper-neck width relative to the middle/lower body."""
        x, y, width, height = bounds
        if width <= 0 or height < 10:
            return None

        local_mask = np.zeros((height, width), dtype=np.uint8)
        offset = np.array([[[x, y]]], dtype=contour.dtype)
        cv2.drawContours(
            local_mask,
            [contour - offset],
            -1,
            255,
            cv2.FILLED,
        )
        row_widths = np.count_nonzero(local_mask, axis=1)
        neck_widths = row_widths[int(0.08 * height):int(0.30 * height)]
        body_widths = row_widths[int(0.45 * height):int(0.85 * height)]
        neck_widths = neck_widths[neck_widths > 0]
        body_widths = body_widths[body_widths > 0]
        if neck_widths.size == 0 or body_widths.size == 0:
            return None

        body_width = float(np.median(body_widths))
        if body_width <= 0.0:
            return None
        return float(np.median(neck_widths)) / body_width
