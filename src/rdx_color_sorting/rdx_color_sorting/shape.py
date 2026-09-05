"""Pure geometry checks used by the color detector."""

from __future__ import annotations


def matches_target_shape(
    *,
    contour_area: float,
    hull_area: float,
    area_ratio: float,
    box_width: int,
    box_height: int,
    minimum_width_height_ratio: float,
    maximum_width_height_ratio: float,
    minimum_extent: float,
    minimum_solidity: float,
    minimum_area_ratio: float,
    maximum_area_ratio: float,
) -> bool:
    """Return whether a contour resembles an upright solid target."""
    if contour_area <= 0.0 or hull_area <= 0.0:
        return False
    if box_width <= 0 or box_height <= 0:
        return False

    width_height_ratio = box_width / box_height
    extent = contour_area / (box_width * box_height)
    solidity = contour_area / hull_area
    return (
        minimum_width_height_ratio
        <= width_height_ratio
        <= maximum_width_height_ratio
        and minimum_area_ratio <= area_ratio <= maximum_area_ratio
        and extent >= minimum_extent
        and solidity >= minimum_solidity
    )


def matches_bottle_shape(
    *,
    contour_area: float,
    hull_area: float,
    area_ratio: float,
    box_width: int,
    box_height: int,
    neck_body_width_ratio: float,
    minimum_width_height_ratio: float,
    maximum_width_height_ratio: float,
    minimum_extent: float,
    minimum_solidity: float,
    maximum_neck_body_width_ratio: float,
    minimum_area_ratio: float,
    maximum_area_ratio: float,
) -> bool:
    """Return whether a contour resembles an upright bottle."""
    if contour_area <= 0.0 or hull_area <= 0.0:
        return False
    if box_width <= 0 or box_height <= 0:
        return False
    if neck_body_width_ratio <= 0.0:
        return False

    width_height_ratio = box_width / box_height
    extent = contour_area / (box_width * box_height)
    solidity = contour_area / hull_area
    return (
        minimum_width_height_ratio
        <= width_height_ratio
        <= maximum_width_height_ratio
        and minimum_area_ratio <= area_ratio <= maximum_area_ratio
        and extent >= minimum_extent
        and solidity >= minimum_solidity
        and neck_body_width_ratio <= maximum_neck_body_width_ratio
    )
