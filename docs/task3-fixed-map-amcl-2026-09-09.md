# Task 3 fixed-map AMCL localization

## Decision

Task 3 now uses two distinct maps:

- one immutable occupancy grid for localization;
- one replaceable cylinder snapshot for sorting.

The occupancy grid is captured exactly once after the initial inventory is
locked, or after the 5-second collection deadline has produced at least one
validated target. GMapping is then stopped from processing scans and from
broadcasting `map -> odom`; AMCL takes sole ownership of that transform. After
each cylinder is delivered, the remaining-cylinder snapshot is rebuilt in the
same fixed `map` frame, but the AMCL occupancy grid is not rebuilt.

Updating the occupancy grid after every push was rejected because it would
change the localization reference while the task is running and reintroduce
the translation/yaw discontinuities seen in the September 9 bags.

## Automatic chain

1. `map_gmapping_launch.py` starts the base, LiDAR, odometry, TF, and GMapping.
2. `task3_simple.launch.py` starts perception, snapshot construction, an
   inactive-until-mapped AMCL, the handoff node, and the path controller.
3. `/simple_task3/run_all` resets perception and starts the initial inventory.
4. When all configured cylinders are validated and locked, the handoff node
   starts automatically. If the 5-second deadline is reached first, the
   controller requests the same handoff using the validated subset. The node:
   - copies the latest `/map` and `map -> base_footprint` pose;
   - atomically saves `task3_initial.pgm` and `task3_initial.yaml`;
   - calls `/slam_gmapping/set_tracking_enabled` with `false`;
   - publishes the captured map as transient-local `/task3/fixed_map`;
   - publishes the captured robot pose on `/initialpose`;
   - waits for a consistent `/amcl_pose`, then publishes `ready=true`.
5. The controller plans and moves only after this ready state.
6. At the end of each push, only the cylinder inventory and exclusions are
   changed. Raw `/scan`, wheel/IMU-fused `/odom`, and AMCL continue running.
7. Later collections also wait up to 5 seconds. On timeout, every object in
   the `validated` state may enter the route planner, regardless of configured
   inventory counts. Extra objects receive additional same-color destination
   slots. After at least one delivery, an empty 5-second window completes the
   open task; at initial startup, an empty window continues collecting.
   A `validated` color remains eligible when its hinted count is zero as long
   as that color has a configured destination; colors without a destination
   remain ineligible.

There must never be two live Task 3 launches, and GMapping and AMCL must never
simultaneously broadcast `map -> odom`.

## Static acceptance before floor motion

With six unobstructed cylinders and motion disabled, call:

```bash
ros2 service call /simple_task3/prepare std_srvs/srv/Trigger '{}'
ros2 topic echo /simple_task3/localization_status --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic info /tf --verbose
ls -l /home/sunrise/task3_maps/task3_initial.{yaml,pgm}
```

Acceptance criteria:

- the initial validated snapshot is locked with the configured inventory;
- localization reports `fixed_amcl`, `ready=true`, and owner `amcl`;
- GMapping no longer advances scan matching or publishes TF;
- `/task3/fixed_map`, `/amcl_pose`, and `map -> base_footprint` continue;
- after moving one cylinder by hand and rebuilding the reduced snapshot, the
  fixed map metadata and saved-file checksum remain unchanged.

Only after those checks should `enable_motion:=true` be used. Start with a
lifted-wheel command-output test, then one cylinder, and finally `run_all` for
six cylinders with an operator next to power control.

## Deployment record

Deployed to `/home/sunrise/yahboomcar_ws` on 2026-09-09. Both
`slam_gmapping` and `simple_task3` built successfully. The robot-side package
tests passed (18 tests), and an isolated-domain service check proved that the
patched GMapping process can disable and restore scan matching plus TF
publication. A motion-disabled live launch brought up all sensor, mapping,
perception, AMCL, handoff, and controller nodes successfully.

The live scene available during deployment produced four LiDAR candidates, so
the initial six-object lock correctly remained incomplete and the handoff did
not run early. The complete six-object handoff and motion test is therefore the
next real-robot acceptance step.

The pre-deployment source backup is:

```text
/home/sunrise/vendor-backups/2026-09-09-before-task3-fixed-amcl/task3-fixed-amcl-source.tar.gz
SHA256 8a77367401f20b76f84ac1c6b734eb33267ca724c9b6e8bf71aeff67a5e75615
```
