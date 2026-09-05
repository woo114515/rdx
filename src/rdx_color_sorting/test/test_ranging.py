import math
import unittest

from rdx_color_sorting.ranging import range_near_bearing


class TargetRangingTest(unittest.TestCase):
    def test_returns_median_range_near_camera_bearing(self) -> None:
        ranges = [math.inf] * 9
        ranges[3:6] = [0.52, 0.50, 0.48]
        distance = range_near_bearing(
            ranges,
            angle_min=-0.4,
            angle_increment=0.1,
            range_min=0.05,
            range_max=8.0,
            bearing=0.0,
            half_angle=0.11,
            minimum_samples=3,
        )
        self.assertAlmostEqual(distance, 0.50)

    def test_returns_none_without_enough_valid_samples(self) -> None:
        distance = range_near_bearing(
            [math.inf, 0.3, math.nan],
            angle_min=-0.1,
            angle_increment=0.1,
            range_min=0.05,
            range_max=8.0,
            bearing=0.0,
            half_angle=0.2,
            minimum_samples=2,
        )
        self.assertIsNone(distance)


if __name__ == "__main__":
    unittest.main()
