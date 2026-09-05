import math
import unittest

from rdx_color_sorting.behavior import VelocityCommand
from rdx_color_sorting.safety import (
    CommandWatchdog,
    ObstacleConfig,
    SafetyConfig,
    command_has_obstacle,
    navigation_controls_output,
    output_topic_has_conflict,
)


class CommandWatchdogTest(unittest.TestCase):
    def test_navigation_controls_only_the_return_phase(self) -> None:
        self.assertFalse(navigation_controls_output(False, "return"))
        self.assertFalse(navigation_controls_output(True, "push"))
        self.assertTrue(navigation_controls_output(True, "return"))

    def test_detects_another_output_publisher(self) -> None:
        self.assertFalse(output_topic_has_conflict(1))
        self.assertTrue(output_topic_has_conflict(2))

    def test_lidar_blocks_obstacle_in_movement_direction(self) -> None:
        ranges = [1.0] * 9
        ranges[4] = 0.2
        blocked = command_has_obstacle(
            VelocityCommand(linear_x=0.1),
            ranges,
            angle_min=-0.4,
            angle_increment=0.1,
            range_min=0.05,
            range_max=8.0,
            config=ObstacleConfig(stop_distance=0.3),
        )
        self.assertTrue(blocked)

    def test_lidar_ignores_narrow_forward_target_corridor(self) -> None:
        ranges = [1.0] * 9
        ranges[4] = 0.2
        blocked = command_has_obstacle(
            VelocityCommand(linear_x=0.1),
            ranges,
            angle_min=-0.4,
            angle_increment=0.1,
            range_min=0.05,
            range_max=8.0,
            config=ObstacleConfig(
                stop_distance=0.3,
                target_corridor_half_angle=0.15,
            ),
            ignore_forward_target_corridor=True,
        )
        self.assertFalse(blocked)

    def test_lidar_blocks_return_direction_and_ignores_invalid_ranges(self) -> None:
        ranges = [float("nan"), float("inf"), 1.0, 1.0, 0.2]
        blocked = command_has_obstacle(
            VelocityCommand(linear_x=-0.1),
            ranges,
            angle_min=0.0,
            angle_increment=math.pi / 4.0,
            range_min=0.05,
            range_max=8.0,
            config=ObstacleConfig(stop_distance=0.3),
        )
        self.assertTrue(blocked)

    def test_stops_without_request_and_after_timeout(self) -> None:
        watchdog = CommandWatchdog(SafetyConfig(request_timeout=0.3))
        self.assertEqual(watchdog.output(0.0), VelocityCommand())
        watchdog.update(0.0, VelocityCommand(linear_x=0.05))
        self.assertEqual(watchdog.output(0.2).linear_x, 0.05)
        self.assertEqual(watchdog.output(0.31), VelocityCommand())

    def test_limits_translation_vector_and_rotation(self) -> None:
        watchdog = CommandWatchdog(
            SafetyConfig(
                maximum_linear_speed=0.10,
                maximum_angular_speed=0.20,
            )
        )
        watchdog.update(
            0.0,
            VelocityCommand(linear_x=0.3, linear_y=0.4, angular_z=0.5),
        )
        command = watchdog.output(0.1)
        self.assertAlmostEqual(
            math.hypot(command.linear_x, command.linear_y),
            0.10,
        )
        self.assertEqual(command.angular_z, 0.20)

    def test_emergency_stop_latches_until_restart(self) -> None:
        watchdog = CommandWatchdog()
        watchdog.update(0.0, VelocityCommand(linear_x=0.05))
        watchdog.emergency_stop()
        watchdog.update(0.1, VelocityCommand(linear_x=0.05))
        self.assertEqual(watchdog.output(0.2), VelocityCommand())


if __name__ == "__main__":
    unittest.main()
