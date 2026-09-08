# GMapping map update stall, 2026-09-08

## Observed behavior

During the Task 3 six-cylinder test, RViz showed live `/scan` returns displaced
from black `/map` cells. Read-only diagnostics found:

- `/scan` remained near 10 Hz and had a current timestamp;
- `/odom` remained near 7.7 Hz, although with scheduling jitter;
- `map -> lidar_link` remained available;
- `/map` did not publish during a 12-second observation and its header was
  approximately 286 seconds older than `/scan`;
- the `slam_gmapping` parameter service stopped responding;
- the process consumed about 64 percent of one CPU according to `ps`; and
- the 8-core board retained roughly 36 percent aggregate idle CPU and ample
  memory, so this was not an out-of-memory failure or complete CPU saturation.

The black occupancy grid and live scan are not expected to be identical after
objects move: GMapping represents accumulated occupancy, whereas `/scan` is a
current measurement. The multi-minute map timestamp lag was nevertheless an
implementation fault, not normal occupancy persistence.

## Root cause in vendor source

The active source was:

```text
/home/sunrise/yahboomcar_ws/src/slam_gmapping/src/slam_gmapping.cpp
/home/sunrise/yahboomcar_ws/src/slam_gmapping/include/slam_gmapping/slam_gmapping.h
```

The installed parameters request `map_update_interval: 5.0` and
`temporalUpdate: 1.0`. However, `laserCallback()` created
`last_map_update = TimePointZero` as a local variable on every invocation.
Whenever GMapping accepted a scan, the elapsed-time test therefore always
passed.

`updateMap()` is expensive: it rebuilds a scan-matcher map by replaying the
best particle's complete trajectory and then traverses every output grid cell.
Calling it without the intended five-second throttle makes callback latency
grow with trajectory history and can eventually leave `/map` far behind live
sensor data.

## Fix and compatibility

The fix adds persistent `last_map_update_` state to `SlamGmapping`, initializes
it once, and updates it only after `updateMap()` returns. The first map remains
immediate through the existing `!got_map_` condition.

The tracked vendor patch is:

```text
reference/vendor-patches/2026-09-08/slam-gmapping-map-update-throttle.patch
```

The change does not modify scan matching parameters, occupancy calculation,
TF, `/map`, `/scan`, `/odom`, `/cmd_vel`, or the base driver. Task 2 navigation
from a saved map uses `map_server`, AMCL, and Nav2 rather than
`slam_gmapping`; it is therefore outside the changed runtime path. Task 2 live
mapping should benefit from lower callback load, with `/map` publication now
limited by the configured five-second interval.

## Verification and rollback

After rebuilding and restarting `map_gmapping_launch.py`, verify:

```bash
timeout 12s ros2 topic hz /map
timeout 6s ros2 topic hz /scan
ros2 topic echo /map --once --field header
ros2 run tf2_ros tf2_echo map lidar_link
```

Expected results are a near-10 Hz scan, a responsive TF chain, and map updates
at roughly 0.2 Hz when GMapping is accepting scans. A moved cylinder can still
leave historical occupied cells; use `/scan` or a Nav2 obstacle layer for live
obstacles rather than treating `/map` as a live sensor view.

Rollback consists of restoring the two deployment backups, rebuilding only
`slam_gmapping`, and restarting the mapping launch.

## Deployment record

The fix was deployed to the robot source workspace on 2026-09-08 and
`slam_gmapping` built successfully. The pre-change files are stored under:

```text
/home/sunrise/vendor-backups/2026-09-08-before-gmapping-throttle/
```

Backup SHA256 values:

```text
925370791e8a829103d71402d787443a2f0e9f7aa8a845b574edc2acebff5cd5  slam_gmapping.cpp
145354266a16cee175fc0654269b89bd3a64b0cf61809c4d14ee47a2665a8aba  slam_gmapping.h
```

The stale pre-build mapping process ignored SIGINT and required termination
after the new binary was installed. Runtime frequency and long-duration
mapping verification remain pending until `map_gmapping_launch.py` is
restarted; no motion command was issued during deployment.
