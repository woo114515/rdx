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
`temporalUpdate: 1.0`. However, the vendor node did not declare or retrieve any
ROS parameters, so the passed YAML file did not change its member variables.
The running process therefore used its hard-coded 0.5-second map interval. In
addition, `laserCallback()` created `last_map_update = TimePointZero` as a local
variable on every invocation. Whenever GMapping accepted a scan, the
elapsed-time test therefore always passed.

`updateMap()` is expensive: it rebuilds a scan-matcher map by replaying the
best particle's complete trajectory and then traverses every output grid cell.
Calling it without the intended five-second throttle makes callback latency
grow with trajectory history and can eventually leave `/map` far behind live
sensor data.

## Fix and compatibility

The first fix added persistent `last_map_update_` state to `SlamGmapping`,
initialized it once, and updated it only after `updateMap()` returned. Further
inspection found two remaining timing defects: the configured timing values
were never loaded, and the conversion used to future-date `map -> odom` cast
the default `0.05` seconds to integer zero.

The completed timing fix declares and reads only `map_update_interval` and
`transform_publish_period`, and constructs the TF offset with
`rclcpp::Duration::from_seconds()`. The first map remains immediate through the
existing `!got_map_` condition. Other GMapping parameters intentionally remain
at their existing hard-coded values until they can be validated separately.

The tracked vendor patch is:

```text
reference/vendor-patches/2026-09-08/slam-gmapping-map-update-throttle.patch
```

The change does not modify scan matching parameters, occupancy calculation,
TF frame topology, `/map`, `/scan`, `/odom`, `/cmd_vel`, or the base driver. It
changes full-grid publication to the configured five-second interval and
correctly applies the intended 50 ms timestamp offset to `map -> odom`. Task 2
navigation from a saved map uses `map_server`, AMCL, and Nav2 rather than
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

The first throttle-state fix was deployed to the robot source workspace on
2026-09-08. The parameter-loading and fractional-duration corrections were
subsequently deployed and the package rebuilt successfully. A standalone,
non-motion startup check confirmed that the running node reads
`map_update_interval=5.0` and `transform_publish_period=0.05` from the installed
YAML file.

The backup for the first deployment is stored under:

```text
/home/sunrise/vendor-backups/2026-09-08-before-gmapping-throttle/
```

Backup SHA256 values:

```text
925370791e8a829103d71402d787443a2f0e9f7aa8a845b574edc2acebff5cd5  slam_gmapping.cpp
145354266a16cee175fc0654269b89bd3a64b0cf61809c4d14ee47a2665a8aba  slam_gmapping.h
```

The backup immediately before the completed timing fix is stored under:

```text
/home/sunrise/vendor-backups/2026-09-08-before-gmapping-timing-fix/
```

Its SHA256 values are:

```text
1e5e5c3b113f981069697102455c8c8a94ea5abc729e034b457339b7cd2c2e00  slam_gmapping.cpp
46d14ea59d34b128913050e2a8430618a3031dcecf2da29a96b25e9896f9f30d  slam_gmapping.h
```

The stale pre-build mapping process ignored SIGINT and required termination
after the new binary was installed. No motion command was issued during that
deployment or during the completed timing-fix deployment. Full live mapping and
low-speed navigation regression tests remain pending.

## Task 3 motion-time map reconstruction control, 2026-09-09

Later rosbag analysis showed that the full-grid reconstruction could still
stall the accepted-scan callback during a push cycle. Repeating an old
`map -> odom` transform made the controller internally appear to follow its
route while the physical route rotated away from the field.

The vendor patch now also provides
`/slam_gmapping/set_map_updates` (`std_srvs/srv/SetBool`). Disabling the service
guards only `updateMap()`. It deliberately does not bypass `addScan()`, best
particle pose extraction, or the `map_to_odom_` assignment. The service is
serialized with `updateMap()`, so a successful disable response means no
previous grid rebuild is still active. Re-enabling forces a fresh `/map` on the
next accepted scan.

This was an intermediate Task 3 mitigation. Later runs proved that live
GMapping scan matching could still rotate `map -> odom` enough to invalidate a
route, even when grid reconstruction was paused.

## Task 3 fixed-map AMCL handoff, 2026-09-09

Task 3 now uses the additional patch
`reference/vendor-patches/2026-09-09/slam-gmapping-fixed-localization-handoff.patch`.
It adds `/slam_gmapping/set_tracking_enabled`; disabling it stops both scan
processing and GMapping's `map -> odom` broadcast after the initial map and
robot pose have been captured. AMCL then becomes the sole transform owner for
all push cycles. Subsequent cylinder locks update only the object snapshot,
not the localization map.

This path is Task 3 infrastructure only. Task 2 continues using its existing
saved map and AMCL/Nav2 launch and does not call either GMapping control
service.
