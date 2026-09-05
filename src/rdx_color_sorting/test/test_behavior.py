import math
import unittest

from rdx_color_sorting.behavior import (
    ControllerConfig,
    Detection,
    frame_transform,
    Pose2D,
    SortController,
    SortState,
    VelocityCommand,
    transform_pose,
    Zone,
)


def make_controller(**overrides):
    overrides.setdefault("minimum_detection_duration", 0.0)
    config = ControllerConfig(**overrides)
    return SortController(
        zones={
            "green": Zone(1.0, 0.5),
            "blue": Zone(1.0, 0.0),
            "orange": Zone(1.0, -0.5),
        },
        targets_per_color={"green": 1, "blue": 1, "orange": 1},
        color_order=("green", "blue", "orange"),
        config=config,
    )


class SortControllerTest(unittest.TestCase):
    def test_map_to_odom_transform_reproduces_amcl_pose(self):
        odom_pose = Pose2D(2.0, -1.0, math.pi / 2.0)
        map_pose = Pose2D(5.0, 3.0, math.pi)
        transform = frame_transform(map_pose, odom_pose)
        transformed = transform_pose(transform, odom_pose)
        self.assertAlmostEqual(transformed.x, map_pose.x)
        self.assertAlmostEqual(transformed.y, map_pose.y)
        self.assertAlmostEqual(transformed.yaw, map_pose.yaw)

    def test_search_rotates_until_current_color_is_seen(self):
        controller = make_controller()
        command = controller.update(0.0, (), Pose2D(0.0, 0.0, 0.0))
        self.assertEqual(controller.state, SortState.SEARCH)
        self.assertGreater(command.angular_z, 0.0)

        command = controller.update(
            0.1,
            (Detection("green", x_error=0.3, area_ratio=0.02),),
            Pose2D(0.0, 0.0, 0.0),
        )
        self.assertEqual(controller.state, SortState.ALIGN)
        self.assertEqual(command.linear_x, 0.0)

    def test_search_selects_largest_available_color(self):
        controller = make_controller()
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(
            0.0,
            (
                Detection("green", x_error=0.1, area_ratio=0.01),
                Detection("blue", x_error=-0.2, area_ratio=0.04),
                Detection("orange", x_error=0.3, area_ratio=0.02),
            ),
            pose,
        )
        self.assertEqual(controller.current_color, "blue")
        self.assertEqual(controller.state, SortState.ALIGN)

    def test_search_requires_stable_detection(self):
        controller = make_controller(minimum_detection_duration=0.25)
        pose = Pose2D(0.0, 0.0, 0.0)
        detection = (Detection("green", x_error=0.0, area_ratio=0.02),)
        command = controller.update(0.0, detection, pose)
        self.assertEqual(controller.state, SortState.SEARCH)
        self.assertEqual(command, VelocityCommand())

        controller.update(0.1, (), pose)
        controller.update(0.2, detection, pose)
        controller.update(0.4, detection, pose)
        self.assertEqual(controller.state, SortState.SEARCH)
        controller.update(0.46, detection, pose)
        self.assertEqual(controller.state, SortState.ALIGN)

    def test_search_ignores_color_with_no_remaining_targets(self):
        controller = SortController(
            zones={
                "green": Zone(1.0, 0.5),
                "blue": Zone(1.0, 0.0),
                "orange": Zone(1.0, -0.5),
            },
            targets_per_color={"green": 0, "blue": 1, "orange": 1},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(minimum_detection_duration=0.0),
        )
        controller.update(
            0.0,
            (
                Detection("green", x_error=0.0, area_ratio=0.20),
                Detection("orange", x_error=0.0, area_ratio=0.02),
            ),
            Pose2D(0.0, 0.0, 0.0),
        )
        self.assertEqual(controller.current_color, "orange")
        self.assertEqual(controller.state, SortState.ALIGN)

    def test_alignment_turns_toward_target_and_approach_is_slow(self):
        controller = make_controller()
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (Detection("green", 0.4, 0.02),), pose)
        command = controller.update(
            0.1,
            (Detection("green", 0.4, 0.02),),
            pose,
        )
        self.assertLess(command.angular_z, 0.0)
        self.assertEqual(command.linear_x, 0.0)

        command = controller.update(
            0.2,
            (Detection("green", 0.01, 0.02),),
            pose,
        )
        self.assertEqual(controller.state, SortState.APPROACH)
        self.assertEqual(command.linear_x, controller.config.approach_speed)

    def test_lidar_range_slows_approach_and_detects_contact(self):
        controller = make_controller(
            approach_speed=0.08,
            minimum_approach_speed=0.03,
            approach_range_kp=0.4,
            contact_distance=0.24,
        )
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(
            0.0,
            (Detection("green", 0.0, 0.02, range_m=0.50),),
            pose,
        )
        command = controller.update(
            0.1,
            (Detection("green", 0.0, 0.02, range_m=0.30),),
            pose,
        )
        self.assertEqual(command.linear_x, 0.03)

        command = controller.update(
            0.2,
            (Detection("green", 0.0, 0.02, range_m=0.23),),
            pose,
        )
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.PUSH)

    def test_lost_target_stops_before_resuming_search(self):
        controller = make_controller(target_lost_timeout=0.5)
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (Detection("green", 0.2, 0.02),), pose)
        command = controller.update(0.2, (), pose)
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

        command = controller.update(0.7, (), pose)
        self.assertEqual(controller.state, SortState.SEARCH)
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_search_timeout_enters_fault_and_stops(self):
        controller = make_controller(
            maximum_search_duration=1.0,
            search_timeout_is_fault=True,
        )
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (), pose)
        command = controller.update(1.0, (), pose)
        self.assertEqual(controller.state, SortState.FAULT)
        self.assertEqual(controller.fault_reason, "search timeout")
        self.assertEqual(command, VelocityCommand())

    def test_unknown_count_finishes_after_no_target_duration(self):
        controller = SortController(
            zones={
                "green": Zone(1.0, 0.5),
                "blue": Zone(1.0, 0.0),
                "orange": Zone(1.0, -0.5),
            },
            targets_per_color={"green": -1, "blue": -1, "orange": -1},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(
                finish_when_no_target_duration=1.0,
                minimum_detection_duration=0.0,
            ),
        )
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (), pose)
        command = controller.update(1.0, (), pose)
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.COMPLETE)

    def test_unknown_count_resets_no_target_timer_when_seen(self):
        controller = SortController(
            zones={
                "green": Zone(1.0, 0.5),
                "blue": Zone(1.0, 0.0),
                "orange": Zone(1.0, -0.5),
            },
            targets_per_color={"green": -1, "blue": -1, "orange": -1},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(
                finish_when_no_target_duration=1.0,
                minimum_detection_duration=1.0,
                maximum_search_duration=5.0,
            ),
        )
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (), pose)
        controller.update(
            0.8,
            (Detection("green", 0.0, 0.02),),
            pose,
        )
        controller.update(0.9, (), pose)
        controller.update(1.2, (), pose)
        self.assertEqual(controller.state, SortState.SEARCH)
        controller.update(1.91, (), pose)
        self.assertEqual(controller.state, SortState.COMPLETE)

    def test_unknown_count_stays_available_and_tracks_processed(self):
        controller = SortController(
            zones={
                "green": Zone(1.0, 0.5),
                "blue": Zone(1.0, 0.0),
                "orange": Zone(1.0, -0.5),
            },
            targets_per_color={"green": -1, "blue": 0, "orange": 0},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(
                release_duration=0.1,
                minimum_detection_duration=0.0,
            ),
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        zone = Pose2D(1.0, 0.5, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), zone)
        controller.update(0.4, (), zone)
        self.assertEqual(controller.remaining["green"], -1)
        self.assertEqual(controller.processed["green"], 1)
        self.assertEqual(controller.state, SortState.RETURN)

    def test_push_timeout_enters_fault_and_stops(self):
        controller = make_controller(maximum_push_duration=1.0)
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), pose)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), pose)
        command = controller.update(1.1, (), pose)
        self.assertEqual(controller.state, SortState.FAULT)
        self.assertEqual(controller.fault_reason, "push timeout")
        self.assertEqual(command, VelocityCommand())

    def test_lost_target_while_pushing_enters_fault(self):
        controller = make_controller(push_target_lost_timeout=0.5)
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), pose)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), pose)
        command = controller.update(0.61, (), pose)
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.FAULT)
        self.assertEqual(
            controller.fault_reason,
            "target lost while pushing",
        )

    def test_release_requires_stable_delivery_confirmation(self):
        controller = make_controller(
            release_duration=0.1,
            delivery_confirmation_timeout=1.0,
            delivery_confirmation_duration=0.25,
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        zone = Pose2D(1.0, 0.5, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), zone)
        delivered = (Detection("green", 0.1, 0.08),)
        controller.update(0.4, delivered, zone)
        controller.update(0.66, delivered, zone)
        self.assertEqual(controller.state, SortState.RETURN)
        self.assertEqual(controller.remaining["green"], 0)

    def test_release_confirmation_timeout_enters_fault(self):
        controller = make_controller(
            release_duration=0.1,
            delivery_confirmation_timeout=0.5,
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        zone = Pose2D(1.0, 0.5, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), zone)
        still_attached = (Detection("green", 0.0, 0.19),)
        controller.update(0.4, still_attached, zone)
        command = controller.update(0.81, still_attached, zone)
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.FAULT)
        self.assertEqual(controller.fault_reason, "delivery not confirmed")

    def test_push_uses_odom_and_releases_at_zone(self):
        controller = make_controller(release_duration=0.5)
        origin = Pose2D(2.0, 3.0, math.pi / 2.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        self.assertEqual(controller.state, SortState.PUSH)

        command = controller.update(0.2, (), origin)
        self.assertGreater(math.hypot(command.linear_x, command.linear_y), 0.0)

        green_zone_in_odom = Pose2D(1.5, 4.0, math.pi / 2.0)
        command = controller.update(0.3, (), green_zone_in_odom)
        self.assertEqual(controller.state, SortState.RELEASE)
        self.assertEqual(command.linear_x, 0.0)

        command = controller.update(0.4, (), green_zone_in_odom)
        self.assertLess(command.linear_x, 0.0)
        controller.update(0.9, (), green_zone_in_odom)
        self.assertEqual(controller.current_color, "blue")
        self.assertEqual(controller.remaining["green"], 0)
        self.assertEqual(controller.state, SortState.RETURN)

        command = controller.update(1.0, (), green_zone_in_odom)
        self.assertGreater(math.hypot(command.linear_x, command.linear_y), 0.0)
        first_corridor_point = Pose2D(1.2, 4.0, math.pi / 2.0)
        second_corridor_point = Pose2D(1.2, 3.0, math.pi / 2.0)
        controller.update(1.1, (), first_corridor_point)
        controller.update(1.2, (), second_corridor_point)
        command = controller.update(1.3, (), origin)
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.SEARCH)

    def test_return_uses_side_corridor_before_origin(self):
        controller = make_controller(release_duration=0.1)
        origin = Pose2D(0.0, 0.0, 0.0)
        green_zone = Pose2D(1.0, 0.5, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), green_zone)
        controller.update(0.4, (), green_zone)

        command = controller.update(0.5, (), green_zone)
        self.assertAlmostEqual(command.linear_x, 0.0)
        self.assertGreater(command.linear_y, 0.0)

        corridor_at_zone = Pose2D(1.0, 0.8, 0.0)
        command = controller.update(0.6, (), corridor_at_zone)
        self.assertLess(command.linear_x, 0.0)
        self.assertAlmostEqual(command.linear_y, 0.0)

    def test_missing_odom_at_contact_enters_fault_and_stops(self):
        controller = make_controller()
        controller.update(0.0, (Detection("green", 0.0, 0.02),), None)
        command = controller.update(
            0.1,
            (Detection("green", 0.0, 0.20),),
            None,
        )
        self.assertEqual(controller.state, SortState.FAULT)
        self.assertEqual(
            controller.fault_reason,
            "odometry unavailable at contact",
        )
        self.assertEqual(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_search_timeout_starts_another_scan_by_default(self):
        controller = make_controller(maximum_search_duration=1.0)
        pose = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (), pose)
        command = controller.update(1.0, (), pose)
        self.assertEqual(controller.state, SortState.SEARCH)
        self.assertGreater(command.angular_z, 0.0)
        self.assertIsNone(controller.fault_reason)

    def test_multiple_targets_rotate_through_colors(self):
        controller = SortController(
            zones={
                "green": Zone(1.0, 0.5),
                "blue": Zone(1.0, 0.0),
                "orange": Zone(1.0, -0.5),
            },
            targets_per_color={"green": 2, "blue": 2, "orange": 2},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(
                release_duration=0.1,
                minimum_detection_duration=0.0,
            ),
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), Pose2D(1.0, 0.5, 0.0))
        controller.update(0.4, (), Pose2D(1.0, 0.5, 0.0))
        self.assertEqual(controller.current_color, "blue")
        self.assertEqual(controller.remaining["green"], 1)
        self.assertEqual(controller.state, SortState.RETURN)

    def test_return_timeout_enters_fault_and_stops(self):
        controller = make_controller(
            release_duration=0.1,
            maximum_return_duration=1.0,
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        zone = Pose2D(1.0, 0.5, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), zone)
        controller.update(0.4, (), zone)
        command = controller.update(1.41, (), zone)
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.FAULT)
        self.assertEqual(controller.fault_reason, "return timeout")

    def test_absolute_map_zone_and_return_use_recorded_map_origin(self):
        controller = SortController(
            zones={
                "green": Zone(1.5, 4.0),
                "blue": Zone(2.0, 4.0),
                "orange": Zone(2.5, 4.0),
            },
            targets_per_color={"green": 1, "blue": 0, "orange": 0},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(
                release_duration=0.1,
                minimum_detection_duration=0.0,
            ),
            zones_relative_to_origin=False,
        )
        origin = Pose2D(2.0, 3.0, math.pi / 2.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)

        command = controller.update(0.2, (), origin)
        self.assertGreater(math.hypot(command.linear_x, command.linear_y), 0.0)
        zone = Pose2D(1.5, 4.0, math.pi / 2.0)
        controller.update(0.3, (), zone)
        self.assertEqual(controller.state, SortState.RELEASE)
        controller.update(0.5, (), zone)
        self.assertEqual(controller.state, SortState.RETURN)

        first_corridor_point = Pose2D(1.2, 4.0, math.pi / 2.0)
        second_corridor_point = Pose2D(1.2, 3.0, math.pi / 2.0)
        controller.update(0.6, (), first_corridor_point)
        controller.update(0.7, (), second_corridor_point)
        command = controller.update(0.8, (), origin)
        self.assertEqual(command, VelocityCommand())
        self.assertEqual(controller.state, SortState.COMPLETE)

    def test_external_return_waits_for_navigation_completion(self):
        controller = SortController(
            zones={
                "green": Zone(1.0, 0.5),
                "blue": Zone(1.0, 0.0),
                "orange": Zone(1.0, -0.5),
            },
            targets_per_color={"green": 1, "blue": 0, "orange": 0},
            color_order=("green", "blue", "orange"),
            config=ControllerConfig(
                release_duration=0.1,
                minimum_detection_duration=0.0,
            ),
            external_return_enabled=True,
        )
        origin = Pose2D(0.0, 0.0, 0.0)
        zone = Pose2D(1.0, 0.5, 0.0)
        controller.update(0.0, (Detection("green", 0.0, 0.02),), origin)
        controller.update(0.1, (Detection("green", 0.0, 0.20),), origin)
        controller.update(0.2, (), zone)
        controller.update(0.4, (), zone)

        self.assertEqual(controller.state, SortState.RETURN)
        self.assertEqual(controller.update(0.5, (), zone), VelocityCommand())
        controller.complete_external_return(0.6)
        self.assertEqual(controller.state, SortState.COMPLETE)


if __name__ == "__main__":
    unittest.main()
