# Mapping and Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS 2 Humble stack that safely maps the fixed arena, loads the saved map for Nav2 localization/navigation, and visits task points 1, 2, 3, then the start point.

**Architecture:** Four focused Python/launch packages separate hardware bringup, velocity safety, SLAM/Nav2 configuration, and mission sequencing. Vendor drivers remain installed dependencies; only the safety node publishes the real `/cmd_vel`. Pure decision logic and YAML validation remain ROS-independent so they can be tested on the development computer.

**Tech Stack:** ROS 2/TROS Humble, Python 3, rclpy, Nav2 1.1.16, SLAM Toolbox 2.6.8, PyYAML, pytest, ament_python.

## Global Constraints

- Target RDK X5 runs Ubuntu 22.04 and ROS 2/TROS Humble with `ROS_DOMAIN_ID=99`.
- Do not download or replace Nav2, SLAM Toolbox, or vendor packages already installed on the robot.
- Do not modify `/home/sunrise/yahboomcar_ws`; depend on its installed overlay.
- All real nodes use `use_sim_time: false`.
- Vendor joy/teleop launch files are excluded; teleop must publish `/cmd_vel_teleop`.
- Nav2 publishes `/cmd_vel_nav`; only `rdx_safety` publishes `/cmd_vel`.
- Default linear speed is capped at 0.18 m/s and angular speed at 0.6 rad/s.
- A command older than 0.25 s or scan older than 0.30 s results in continuous zero velocity.
- Navigation remains locked until the measured footprint is recorded and `footprint_verified: true`.
- Automated tests never publish to the physical robot.

## Pause Checkpoint (2026-09-03)

Work is paused at the user's request on branch `feat/navigation-stack` in
`/home/zyw/x5/rdx/.worktrees/navigation-stack`.

Completed:

- created and committed this implementation plan;
- implemented the ROS-independent velocity safety core;
- verified 19 unit tests covering timeouts, emergency stop, unverified footprint, directional
  obstacles, slowdown, speed limiting, invalid scans, and invalid commands;
- created the `rdx_safety` ROS package, fail-closed YAML defaults, ROS adapter node, and console
  entry point;
- created the `rdx_bringup` package and minimal hardware launch for the verified base, driver,
  robot description, and MS200P packages;
- verified 21 total tests, including configuration defaults and the absence of keyboard/gamepad
  control sources from the minimal launch;
- passed Python syntax compilation for both new ROS package setup files, the safety node, and the
  hardware launch file.

At the pause point, the following items were still pending:

- run `colcon build` for Task 2 and review the installed launch/config layout;
- implement the SLAM Toolbox mapping package and mapping launch (Task 3);
- implement Humble-compatible static-map Nav2 parameters and launch (Task 4);
- implement validated waypoint loading and the fixed-order action client (Task 5);
- write the complete operator runbook and perform repository-wide checks (Task 6);
- validate vendor launch file names, topic/TF behavior, SLAM, and Nav2 on the RDK X5 after charging.

Resume with:

```bash
cd /home/zyw/x5/rdx/.worktrees/navigation-stack
git status --short --branch
PYTHONPATH=src/rdx_safety /usr/bin/python3 -m pytest -q \
  tests/test_safety_logic.py tests/test_bringup_contract.py
```

The development computer has ROS 2 Jazzy but not Nav2 or SLAM Toolbox. The robot already has the
required Humble packages, so no package download is planned. No ROS node or physical motion command
has been run during this implementation session.

---

## File Structure

```text
src/
  rdx_safety/
    package.xml
    setup.cfg
    setup.py
    resource/rdx_safety
    config/safety.yaml
    rdx_safety/__init__.py
    rdx_safety/safety_logic.py
    rdx_safety/safety_node.py
  rdx_bringup/
    package.xml
    setup.cfg
    setup.py
    resource/rdx_bringup
    launch/hardware.launch.py
    rdx_bringup/__init__.py
  rdx_navigation/
    package.xml
    setup.cfg
    setup.py
    resource/rdx_navigation
    config/slam_toolbox.yaml
    config/nav2.yaml
    config/waypoints.yaml
    launch/mapping.launch.py
    launch/navigation.launch.py
    rdx_navigation/__init__.py
  rdx_mission/
    package.xml
    setup.cfg
    setup.py
    resource/rdx_mission
    rdx_mission/__init__.py
    rdx_mission/mission_logic.py
    rdx_mission/mission_node.py
tests/
  test_safety_logic.py
  test_navigation_configs.py
  test_mission_logic.py
docs/
  mapping-navigation-guide.md
```

### Task 1: Safety decision core

**Files:**
- Create: `tests/test_safety_logic.py`
- Create: `src/rdx_safety/rdx_safety/safety_logic.py`

**Interfaces:**
- Produces: immutable `Velocity`, `Scan`, `SafetyConfig`, and `SafetyDecision` data classes.
- Produces: `SafetyController.evaluate(now, nav_command, nav_stamp, teleop_command, teleop_stamp, scan, scan_stamp, emergency_stop, footprint_verified) -> SafetyDecision`.
- Selection order: emergency/verification/sensor gates, then fresh teleop command, then fresh Nav2 command.

- [x] **Step 1: Write failing safety tests**

```python
def test_unverified_footprint_blocks_motion():
    decision = controller().evaluate(
        now=1.0,
        nav_command=Velocity(0.1, 0.0, 0.0),
        nav_stamp=1.0,
        teleop_command=None,
        teleop_stamp=None,
        scan=clear_scan(),
        scan_stamp=1.0,
        emergency_stop=False,
        footprint_verified=False,
    )
    assert decision.velocity == Velocity.zero()
    assert decision.reason == "footprint_unverified"


def test_close_obstacle_stops_translation():
    scan = directional_scan(angle=0.0, distance=0.25)
    decision = controller().evaluate(
        now=1.0,
        nav_command=Velocity(0.1, 0.0, 0.0),
        nav_stamp=1.0,
        teleop_command=None,
        teleop_stamp=None,
        scan=scan,
        scan_stamp=1.0,
        emergency_stop=False,
        footprint_verified=True,
    )
    assert decision.velocity == Velocity.zero()
    assert decision.reason == "obstacle_stop"


def test_mid_zone_obstacle_scales_command():
    scan = directional_scan(angle=0.0, distance=0.475)
    decision = controller().evaluate(
        now=1.0,
        nav_command=Velocity(0.1, 0.0, 0.0),
        nav_stamp=1.0,
        teleop_command=None,
        teleop_stamp=None,
        scan=scan,
        scan_stamp=1.0,
        emergency_stop=False,
        footprint_verified=True,
    )
    assert decision.velocity.linear_x == pytest.approx(0.05)


def test_stale_command_and_scan_each_stop():
    assert evaluate_with(command_age=0.26).reason == "command_timeout"
    assert evaluate_with(scan_age=0.31).reason == "scan_timeout"


def test_teleop_preempts_nav_and_limits_velocity():
    decision = controller().evaluate(
        now=1.0,
        nav_command=Velocity(0.1, 0.0, 0.0),
        nav_stamp=1.0,
        teleop_command=Velocity(1.0, -1.0, 2.0),
        teleop_stamp=1.0,
        scan=clear_scan(),
        scan_stamp=1.0,
        emergency_stop=False,
        footprint_verified=True,
    )
    assert decision.source == "teleop"
    assert decision.velocity == Velocity(0.18, -0.18, 0.6)
```

- [x] **Step 2: Run tests and verify RED**

Run:

```bash
PYTHONPATH=src/rdx_safety /usr/bin/python3 -m pytest -q tests/test_safety_logic.py
```

Expected: collection fails because `rdx_safety.safety_logic` does not exist.

- [x] **Step 3: Implement the minimum safety core**

`SafetyController.evaluate` must:

1. return zero for emergency stop or unverified footprint;
2. choose fresh teleop over fresh navigation;
3. return zero for stale/missing commands or scans;
4. clamp x/y/angular velocity independently;
5. inspect valid finite scan samples in a configurable sector centered on `atan2(y, x)`;
6. stop at or below 0.35 m and linearly scale translation between 0.35 m and 0.60 m;
7. use the nearest all-around sample for pure rotation;
8. return a stable reason string for diagnostics.

- [x] **Step 4: Run tests and verify GREEN**

Run the Step 2 command. Expected: all safety tests pass.

- [x] **Step 5: Commit**

```bash
git add tests/test_safety_logic.py src/rdx_safety/rdx_safety/safety_logic.py
git commit -m "feat: add velocity safety decision core"
```

### Task 2: ROS safety package and minimal hardware bringup

**Files:**
- Create: `src/rdx_safety/package.xml`
- Create: `src/rdx_safety/setup.cfg`
- Create: `src/rdx_safety/setup.py`
- Create: `src/rdx_safety/resource/rdx_safety`
- Create: `src/rdx_safety/config/safety.yaml`
- Create: `src/rdx_safety/rdx_safety/__init__.py`
- Create: `src/rdx_safety/rdx_safety/safety_node.py`
- Create: `src/rdx_bringup/package.xml`
- Create: `src/rdx_bringup/setup.cfg`
- Create: `src/rdx_bringup/setup.py`
- Create: `src/rdx_bringup/resource/rdx_bringup`
- Create: `src/rdx_bringup/launch/hardware.launch.py`
- Create: `src/rdx_bringup/rdx_bringup/__init__.py`
- Modify: `tests/test_safety_logic.py`

**Interfaces:**
- Consumes: `/cmd_vel_nav`, `/cmd_vel_teleop`, `/scan`, and `/emergency_stop`.
- Publishes: `/cmd_vel`, `/rdx_safety/state`, and `/rdx_safety/ready`.
- Hardware launch starts `base_node`, `Mcnamu_driver`, vendor robot description, MS200P, and `rdx_safety_node`; it starts no joystick or keyboard node.

- [x] **Step 1: Add failing source-selection and shutdown tests**

```python
def test_zero_command_does_not_require_scan():
    decision = evaluate_with(command=Velocity.zero(), scan=None, footprint_verified=True)
    assert decision.velocity == Velocity.zero()
    assert decision.reason == "idle"


def test_emergency_stop_has_highest_priority():
    decision = evaluate_with(
        command=Velocity(0.1, 0.0, 0.0),
        emergency_stop=True,
        footprint_verified=True,
    )
    assert decision.reason == "emergency_stop"
```

- [x] **Step 2: Verify RED, implement, then verify GREEN**

Run the Task 1 test command before changing `SafetyController.evaluate`, then run it again after
adding the idle and emergency-stop branches.

- [x] **Step 3: Add the ROS node**

`SafetyNode` stores the most recent messages using the monotonic ROS clock, evaluates at 20 Hz,
publishes zero continuously while blocked, and publishes three zero commands during normal teardown.
Laser scans use sensor-data QoS. All topic names and thresholds are ROS parameters loaded from
`config/safety.yaml`; `footprint_verified` defaults to false.

- [x] **Step 4: Add minimal vendor launch**

`hardware.launch.py` starts:

```text
yahboomcar_base_node/base_node (pub_odom_tf=true)
yahboomcar_bringup/Mcnamu_driver
yahboomcar_description/description_launch.py
oradar_lidar_ms200/ms200_scan.launch.py
rdx_safety/rdx_safety_node
```

It remaps both `/MS200/scan` and relative `scan` to `/scan`. It exposes launch arguments
`start_driver`, `start_lidar`, and `footprint_verified`, all defaulting to safe values.

- [ ] **Step 5: Build and inspect**

Run:

```bash
colcon build --symlink-install --packages-select rdx_safety rdx_bringup
/usr/bin/python3 -m compileall -q src/rdx_safety src/rdx_bringup
```

Expected: both packages build and Python compilation succeeds. No node is launched.

- [ ] **Step 6: Commit**

```bash
git add src/rdx_safety src/rdx_bringup tests/test_safety_logic.py
git commit -m "feat: add safe robot hardware bringup"
```

### Task 3: SLAM Toolbox mapping launch

**Files:**
- Create: `tests/test_navigation_configs.py`
- Create: `src/rdx_navigation/package.xml`
- Create: `src/rdx_navigation/setup.cfg`
- Create: `src/rdx_navigation/setup.py`
- Create: `src/rdx_navigation/resource/rdx_navigation`
- Create: `src/rdx_navigation/rdx_navigation/__init__.py`
- Create: `src/rdx_navigation/config/slam_toolbox.yaml`
- Create: `src/rdx_navigation/launch/mapping.launch.py`

**Interfaces:**
- Mapping launch optionally includes `rdx_bringup/hardware.launch.py`.
- SLAM consumes `/scan` and `odom -> base_footprint`, publishes `/map` and `map -> odom`.
- Teleoperation remains a separate terminal command remapped to `/cmd_vel_teleop`.

- [x] **Step 1: Write failing configuration tests**

```python
def test_slam_uses_real_time_and_verified_frames():
    params = load_ros_params("src/rdx_navigation/config/slam_toolbox.yaml", "slam_toolbox")
    assert params["use_sim_time"] is False
    assert params["scan_topic"] == "/scan"
    assert params["odom_frame"] == "odom"
    assert params["base_frame"] == "base_footprint"
    assert params["map_frame"] == "map"
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.05
```

- [x] **Step 2: Run and verify RED**

```bash
/usr/bin/python3 -m pytest -q tests/test_navigation_configs.py
```

Expected: missing configuration file.

- [x] **Step 3: Add Humble-compatible SLAM configuration and launch**

Base the parameters on the upstream Humble async mapper, use `scan_queue_size: 1`,
`minimum_travel_distance: 0.15`, `minimum_travel_heading: 0.15`, and retain loop closure.
The launch accepts `start_hardware`, `slam_params_file`, and `footprint_verified`.

- [x] **Step 4: Verify GREEN and build**

```bash
/usr/bin/python3 -m pytest -q tests/test_navigation_configs.py
colcon build --symlink-install --packages-select rdx_navigation
```

- [x] **Step 5: Commit**

```bash
git add src/rdx_navigation tests/test_navigation_configs.py
git commit -m "feat: add safe SLAM mapping launch"
```

### Task 4: Static-map Nav2 launch and conservative parameters

**Files:**
- Create: `src/rdx_navigation/config/nav2.yaml`
- Create: `src/rdx_navigation/launch/navigation.launch.py`
- Modify: `tests/test_navigation_configs.py`

**Interfaces:**
- Launch requires an existing map YAML path and optionally starts hardware.
- Includes `nav2_bringup/bringup_launch.py` with AMCL and lifecycle autostart.
- Remaps Nav2 `/cmd_vel` to `/cmd_vel_nav`.
- Uses `base_footprint`, `odom`, `map`, and `/odom_raw`.

- [x] **Step 1: Add failing Nav2 safety tests**

```python
def test_nav2_is_real_robot_and_routes_velocity_through_safety():
    data = yaml.safe_load(Path(NAV2).read_text())
    assert all_use_sim_time_values_are_false(data)
    assert data["bt_navigator"]["ros__parameters"]["odom_topic"] == "/odom_raw"
    follow = data["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["max_vel_x"] <= 0.18
    assert follow["max_vel_y"] <= 0.18
    assert follow["max_vel_theta"] <= 0.6


def test_costmaps_use_scan_and_conservative_clearance():
    data = yaml.safe_load(Path(NAV2).read_text())
    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["inflation_layer"]["inflation_radius"] >= 0.55
    assert global_["inflation_layer"]["inflation_radius"] >= 0.55
    assert "/scan" in collect_observation_topics(data)
```

- [x] **Step 2: Verify RED, implement params/launch, verify GREEN**

Use the Task 3 pytest command. The initial footprint is a conservative 0.30 m radius, while
`rdx_safety` still blocks movement until the actual body-plus-fork dimensions are verified.
Disable backup and spin recovery plugins for the first field test; retain wait behavior.

- [x] **Step 3: Build and commit**

```bash
colcon build --symlink-install --packages-select rdx_navigation
git add src/rdx_navigation tests/test_navigation_configs.py
git commit -m "feat: add static-map Nav2 configuration"
```

### Task 5: Fixed-order waypoint mission

**Files:**
- Create: `tests/test_mission_logic.py`
- Create: all `src/rdx_mission` package files
- Create: `src/rdx_navigation/config/waypoints.yaml`
- Modify: `src/rdx_navigation/launch/navigation.launch.py`

**Interfaces:**
- `load_mission(path)->MissionPlan` validates `frame_id`, readiness, finite poses, unique names,
  and exact order `task_1, task_2, task_3, start`.
- `MissionStateMachine` emits one goal at a time and ends in `completed`, `failed`, or `cancelled`.
- ROS node provides `/mission/start` and `/mission/cancel` Trigger services and uses
  `nav2_msgs/action/NavigateToPose`.
- Default waypoint file has `ready: false`; the node refuses to move until real RViz poses are saved.

- [x] **Step 1: Write failing parser and state-machine tests**

```python
def test_loads_exact_competition_order(tmp_path):
    plan = load_mission(write_ready_plan(tmp_path))
    assert [point.name for point in plan.waypoints] == [
        "task_1", "task_2", "task_3", "start"
    ]


def test_unready_or_duplicate_plan_is_rejected(tmp_path):
    with pytest.raises(MissionConfigError, match="not ready"):
        load_mission(write_unready_plan(tmp_path))
    with pytest.raises(MissionConfigError, match="duplicate"):
        load_mission(write_duplicate_plan(tmp_path))


def test_failure_does_not_skip_to_next_waypoint():
    machine = MissionStateMachine(ready_plan())
    assert machine.start().name == "task_1"
    machine.goal_failed("blocked")
    assert machine.state == "failed"
    assert machine.next_goal() is None


def test_success_returns_to_start_and_completes():
    machine = MissionStateMachine(ready_plan())
    visited = [machine.start().name]
    while machine.state == "running":
        next_goal = machine.goal_succeeded()
        if next_goal:
            visited.append(next_goal.name)
    assert visited == ["task_1", "task_2", "task_3", "start"]
    assert machine.state == "completed"
```

- [x] **Step 2: Verify RED, implement pure logic, verify GREEN**

```bash
PYTHONPATH=src/rdx_mission /usr/bin/python3 -m pytest -q tests/test_mission_logic.py
```

- [x] **Step 3: Implement ROS action client and service control**

The node starts idle, loads the plan once, waits for Nav2 action availability, sends only the current
goal, cancels on request, and never advances after an aborted/cancelled goal. It logs mission state
and publishes no velocity messages itself.

- [x] **Step 4: Build, run all pure tests, and commit**

```bash
colcon build --symlink-install --packages-select rdx_mission rdx_navigation
PYTHONPATH=src/rdx_safety:src/rdx_mission /usr/bin/python3 -m pytest -q tests
git add src/rdx_mission src/rdx_navigation tests/test_mission_logic.py
git commit -m "feat: add fixed-order navigation mission"
```

### Task 6: Operator guide and static validation

**Files:**
- Create: `docs/mapping-navigation-guide.md`
- Modify: `README.md`
- Modify: `docs/roadmap.md`

**Interfaces:**
- Documents development-machine build, robot deployment, safe mapping, map saving, waypoint entry,
  navigation startup, mission start/cancel, RViz use, and troubleshooting.

- [x] **Step 1: Write the operator guide**

The guide must include these ordered gates:

1. charge the battery and measure the body-plus-fork footprint;
2. discover the current DHCP address without restarting network services;
3. copy/pull the repository and build over the vendor overlay;
4. run hardware launch with `footprint_verified:=false` and verify topics/TF;
5. set measured footprint, then perform wheels-up timeout/emergency-stop tests;
6. map at no more than 0.12 m/s and save the map with `map_saver_cli`;
7. record task 1, task 2, task 3, and start poses into `waypoints.yaml`;
8. start static-map navigation and initialize AMCL;
9. call `/mission/start`; use `/mission/cancel` or emergency stop if needed.

- [ ] **Step 2: Run complete verification**

```bash
git diff --check
PYTHONPATH=src/rdx_safety:src/rdx_mission /usr/bin/python3 -m pytest -q tests
/usr/bin/python3 -m compileall -q src
colcon build --symlink-install
```

Expected: tests, compilation, and build pass without launching robot nodes. Nav2 and SLAM runtime
integration remains an on-robot validation because the development computer lacks those packages.

- [ ] **Step 3: Review for secrets and generated artifacts**

```bash
git status --short
git diff --check
```

Confirm no private keys, maps, rosbag files, `build/`, `install/`, or `log/` are staged.

- [ ] **Step 4: Commit**

```bash
git add README.md docs/mapping-navigation-guide.md docs/roadmap.md
git commit -m "docs: add mapping and navigation runbook"
```
