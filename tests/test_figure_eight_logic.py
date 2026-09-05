import math
from rdx_base_control.figure_eight_logic import Command, Config, Pose, Tracker, make_path

def test_path_is_closed_and_starts_forward():
    path = make_path(Config(), Pose(1.0, 2.0, 0.0))
    assert len(path) == 721
    assert math.hypot(path[-1][0]-1.0, path[-1][1]-2.0) < 1e-9
    assert path[1][0] > path[0][0]
    assert abs(path[1][1]-path[0][1]) < 1e-5

def test_missing_pose_stops_immediately():
    tracker = Tracker()
    tracker.start(Pose(0.0, 0.0, 0.0))
    tracker.update(Pose(0.0, 0.0, 0.0), 0.05)
    assert tracker.update(None, 0.05) == Command()
    assert not tracker.active

def test_commands_obey_limits_and_slew_rate():
    tracker = Tracker()
    tracker.start(Pose(0.0, 0.0, 0.0))
    first = tracker.update(Pose(0.0, 0.0, 0.0), 0.05)
    second = tracker.update(Pose(0.01, 0.0, 0.0), 0.05)
    assert 0.0 < first.linear_x <= 0.04 + 1e-9
    assert second.linear_x-first.linear_x <= 0.04 + 1e-9
    assert abs(first.angular_z) <= 0.1 + 1e-9
    assert abs(second.angular_z) <= 1.0

def test_progress_does_not_jump_to_other_lobe_at_crossing():
    tracker = Tracker()
    tracker.start(Pose(0.0, 0.0, 0.0))
    tracker.index = 350
    tracker.update(Pose(0.0, 0.0, 0.0), 0.05)
    assert 350 <= tracker.index <= 430

def test_goal_stops_and_marks_complete():
    tracker = Tracker()
    tracker.start(Pose(0.0, 0.0, 0.0))
    tracker.index = len(tracker.path)-8
    assert tracker.update(Pose(0.0, 0.0, 0.0), 0.05) == Command()
    assert tracker.completed
    assert not tracker.active
