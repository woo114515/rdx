# Task 3 vendor fixes

## Encoder sample guard

`yahboomcar-encoder-sample-guard.patch` applies to the vendor workspace `src`
directory. It fixes a field failure where the ROS timer read the same encoder
feedback twice, inferred a zero or negative sample interval, and allowed a
`ValueError` to terminate `Mcnamu_driver`.

The patched driver calculates and publishes wheel rates only when the inferred
encoder sample time advances by at least 1 ms. Duplicate, regressing, and
non-finite samples are skipped without changing the previous count/time
baseline. Other fresh feedback continues to publish and the strict low-level
rate calculation still rejects invalid caller input.

Apply only after backing up the source and stopping hardware launch processes:

```bash
cd /home/sunrise/yahboomcar_ws/src
patch -p1 < /path/to/yahboomcar-encoder-sample-guard.patch
colcon build --symlink-install --packages-select yahboomcar_bringup
```

Run `yahboomcar_bringup/test/test_motion_safety.py` before restarting the
hardware launch. No motion command is required for this regression test.

## Navigation constraint

`yahboomcar-nav-no-strafe.patch` applies to the vendor
`yahboomcar_nav` package root. It disables DWB lateral velocity sampling in
`params/dwb_nav_params.yaml`.

Task 3 reobservation changes viewpoint through a short Nav2 path around the
outside of the target group. Direct mecanum strafing is excluded because field
testing showed substantial lateral slip. The reobservation node independently
checks the live `FollowPath.min_vel_y` and `FollowPath.max_vel_y` parameters and
refuses execution unless both are zero.

Apply with:

```bash
cd /home/sunrise/yahboomcar_ws/src/yahboomcar_nav
patch -p1 < /path/to/yahboomcar-nav-no-strafe.patch
```

The source file must be backed up before applying the patch. Restart Nav2 after
the change because controller parameters are read at startup.
