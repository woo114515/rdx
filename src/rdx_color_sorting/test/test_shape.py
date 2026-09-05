import unittest

from rdx_color_sorting.shape import matches_bottle_shape, matches_target_shape


class ShapeFilterTest(unittest.TestCase):
    def test_accepts_upright_solid_target(self) -> None:
        self.assertTrue(
            matches_target_shape(
                contour_area=800.0,
                hull_area=900.0,
                area_ratio=0.02,
                box_width=20,
                box_height=50,
                minimum_width_height_ratio=0.20,
                maximum_width_height_ratio=1.20,
                minimum_extent=0.45,
                minimum_solidity=0.80,
                minimum_area_ratio=0.001,
                maximum_area_ratio=0.35,
            )
        )

    def test_rejects_wide_region(self) -> None:
        self.assertFalse(
            matches_target_shape(
                contour_area=3500.0,
                hull_area=3600.0,
                area_ratio=0.02,
                box_width=100,
                box_height=40,
                minimum_width_height_ratio=0.20,
                maximum_width_height_ratio=1.20,
                minimum_extent=0.45,
                minimum_solidity=0.80,
                minimum_area_ratio=0.001,
                maximum_area_ratio=0.35,
            )
        )

    def test_rejects_sparse_or_irregular_region(self) -> None:
        common = {
            "box_width": 30,
            "box_height": 60,
            "area_ratio": 0.02,
            "minimum_width_height_ratio": 0.20,
            "maximum_width_height_ratio": 1.20,
            "minimum_extent": 0.45,
            "minimum_solidity": 0.80,
            "minimum_area_ratio": 0.001,
            "maximum_area_ratio": 0.35,
        }
        self.assertFalse(
            matches_target_shape(
                contour_area=600.0,
                hull_area=700.0,
                **common,
            )
        )
        self.assertFalse(
            matches_target_shape(
                contour_area=900.0,
                hull_area=1300.0,
                **common,
            )
        )

    def test_rejects_region_that_is_too_large(self) -> None:
        self.assertFalse(
            matches_target_shape(
                contour_area=24000.0,
                hull_area=25000.0,
                area_ratio=0.50,
                box_width=150,
                box_height=200,
                minimum_width_height_ratio=0.20,
                maximum_width_height_ratio=1.20,
                minimum_extent=0.45,
                minimum_solidity=0.80,
                minimum_area_ratio=0.001,
                maximum_area_ratio=0.35,
            )
        )

    def test_accepts_upright_bottle_with_narrow_neck(self) -> None:
        self.assertTrue(
            matches_bottle_shape(
                contour_area=1200.0,
                hull_area=1450.0,
                area_ratio=0.025,
                box_width=30,
                box_height=80,
                neck_body_width_ratio=0.55,
                minimum_width_height_ratio=0.15,
                maximum_width_height_ratio=0.65,
                minimum_extent=0.35,
                minimum_solidity=0.75,
                maximum_neck_body_width_ratio=0.78,
                minimum_area_ratio=0.001,
                maximum_area_ratio=0.35,
            )
        )

    def test_bottle_filter_rejects_uniform_width_target(self) -> None:
        self.assertFalse(
            matches_bottle_shape(
                contour_area=1500.0,
                hull_area=1550.0,
                area_ratio=0.025,
                box_width=30,
                box_height=80,
                neck_body_width_ratio=0.96,
                minimum_width_height_ratio=0.15,
                maximum_width_height_ratio=0.65,
                minimum_extent=0.35,
                minimum_solidity=0.75,
                maximum_neck_body_width_ratio=0.78,
                minimum_area_ratio=0.001,
                maximum_area_ratio=0.35,
            )
        )

    def test_bottle_filter_rejects_irregular_region(self) -> None:
        self.assertFalse(
            matches_bottle_shape(
                contour_area=900.0,
                hull_area=1500.0,
                area_ratio=0.025,
                box_width=30,
                box_height=80,
                neck_body_width_ratio=0.55,
                minimum_width_height_ratio=0.15,
                maximum_width_height_ratio=0.65,
                minimum_extent=0.35,
                minimum_solidity=0.75,
                maximum_neck_body_width_ratio=0.78,
                minimum_area_ratio=0.001,
                maximum_area_ratio=0.35,
            )
        )


if __name__ == "__main__":
    unittest.main()
