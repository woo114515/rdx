# Simple Task 3 Controller

This package is an isolated replacement controller for Task 3. It does not
modify or launch the legacy controller. Existing LiDAR snapshot and visual
validation nodes are reused; every planned and tracked path remains in `map`.
Wheel odometry is used only to measure the short release retreat.

The initial six-cylinder collection runs while GMapping owns `map -> odom`.
When that inventory locks, `localization_handoff` saves the current occupancy
grid, disables both GMapping scan matching and its TF publication, publishes
the captured grid on `/task3/fixed_map`, and initializes AMCL with the robot
pose from the same instant. AMCL then remains the only `map -> odom` owner for
the complete sorting task. Later five/four/... cylinder locks update only the
object snapshot; they never replace the localization map.

The GMapping workspace therefore needs all three patches in chronological
order, ending with
`reference/vendor-patches/2026-09-09/slam-gmapping-fixed-localization-handoff.patch`.

## Services

- `/simple_task3/prepare` resets perception and collects the configured inventory.
- `/simple_task3/run_once` executes one push and starts collecting the remainder.
- `/simple_task3/run_all` prepares and automatically repeats until inventory is zero.
- `/simple_task3/cancel` stops the current cycle and disables automatic repetition.

`run_all` never retries a failed cycle. The deployed configuration enables
`path_only_mode`: planning still rejects routes that intersect the expanded
remaining-cylinder keepout, but execution follows the accepted immutable route
without live obstacle, target-presence, path-deviation, home-distance,
release-drift, SLAM-jump, sensor-timestamp, or phase-timeout vetoes. Missing
`map -> base_footprint` pose, missing release odometry, cancellation, and shutdown
still command zero velocity because continuing would no longer be path following.
Motion is disabled by default and must still be enabled at launch.

## Start

In every robot terminal:

```bash
source /home/sunrise/yahboomcar_ws/task3_ros_env.sh
```

Use separate terminals so every long-running process remains visible.

Terminal 1, base, LiDAR, TF and GMapping:

```bash
ros2 launch yahboomcar_nav map_gmapping_launch.py use_joy:=false
```

Confirm that the patched GMapping handoff service exists:

```bash
ros2 service type /slam_gmapping/set_tracking_enabled
```

It must report `std_srvs/srv/SetBool`. The Task 3 launch performs the one-time
handoff automatically after the initial validated inventory locks.

Terminal 2, camera:

```bash
ros2 run yahboomcar_csi_cam_py yahboomcar_csi_cam
```

Terminal 3, perception and the new controller, initially without motion:

```bash
ros2 launch simple_task3 task3_simple.launch.py enable_motion:=false
```

Terminal 4, prepare and inspect the locked inventory:

```bash
ros2 service call /simple_task3/prepare std_srvs/srv/Trigger '{}'
ros2 topic echo /cylinder_snapshot/validated_objects \
  --qos-reliability reliable --qos-durability transient_local --once
ros2 topic echo /simple_task3/localization_status \
  --qos-reliability reliable --qos-durability transient_local --once
```

Do not enable motion until localization status reports `state=fixed_amcl`,
`ready=true`, `localization_owner=amcl`, and the saved files exist under
`/home/sunrise/task3_maps/`.

Terminal 5, keep status visible:

```bash
ros2 topic echo /simple_task3/status
```

Only after static and lifted-wheel checks may the controller be restarted with
`enable_motion:=true`. Begin real-floor testing with `run_once`, not `run_all`.
Never run `task3_compact.launch.py`, joystick control, keyboard teleoperation,
or Nav2 at the same time.

Before every motion test, verify that the controller is the only `/cmd_vel`
publisher:

```bash
ros2 topic info /cmd_vel --verbose
```

Execute one cycle or the complete inventory with:

```bash
ros2 service call /simple_task3/run_once std_srvs/srv/Trigger '{}'
ros2 service call /simple_task3/run_all std_srvs/srv/Trigger '{}'
```

Keep this cancellation command ready in a dedicated terminal:

```bash
ros2 service call /simple_task3/cancel std_srvs/srv/Trigger '{}'
```
