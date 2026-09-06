# Vendor hardening patch set (source only)

This directory records the source changes reconstructed after the original `/tmp`
working copy was lost. Nothing in this directory is automatically deployed.

## Baseline

The patches target the verified robot backup:

```text
/home/sunrise/vendor-backups/2026-09-04-before-vendor-hardening.tar.gz
SHA256 b776b71f8bc260c67fa1f804522b305ffbc1415ce40c6fb1bc64f48b518db6fa
```

Patch integrity:

```text
ed06143eaeb1ffc7f2dd9e24af9a0a4330ee1b99b24046521a385cc149a949e3  yahboomcar-workspace-hardening.patch
a5ebe85835abe4edaba0adcf7066b4abb3bdc20daf1a39690c5ed357d0d34bec  sunrise-robot-lib-hardening.patch
```

`yahboomcar-workspace-hardening.patch` is applied from
`/home/sunrise/yahboomcar_ws/src`. `sunrise-robot-lib-hardening.patch` is
applied to an unpacked copy of `SunriseRobotLib-3.3.9-py3.10.egg`; the egg must
then be rebuilt and validated before installation. Do not apply either patch
independently on the live robot.

## Behavioural changes

- The driver accepts only the configurable official `/cmd_vel` motion topic, rejects
  non-finite values, bounds commands, stops on stale commands, and exits on an
  unhealthy serial transport.
- Serial reads/writes are bounded to 0.2 seconds. The receive worker and each
  report type expose freshness information; the three direct motion APIs return
  failure instead of silently swallowing transport errors.
- `/vel_raw` remains for compatibility; `/vel_raw_stamped` carries the estimated
  Linux receipt time and `/driver_health` reports transport state. This is not a
  true MCU timestamp.
- `base_node` integrates `/vel_raw_stamped` sample time, rejects invalid gaps,
  normalizes yaw, and publishes conservative covariance.
- Calibration tools publish through `/cmd_vel`, command one degree of
  freedom only, stop on TF loss/timeout, and repeat zero on shutdown.
- Vendor keyboard and joystick publish through `/cmd_vel`; joystick output
  remains disabled until explicitly activated, and both repeat zero on exit.

## Official-course compatibility

The patch keeps the public interfaces used by Yahboom lessons 6710 and 6712 unchanged.
The official `map_gmapping_launch.py`, `yahboom_keyboard`, `save_map_launch.py`,
`laser_bringup_launch.py`, and `navigation_dwb_launch.py` commands therefore do
not require any RDX node. The command watchdog and transport fail-closed behavior
live inside the hardened official driver.

## Required validation before deployment

1. Ensure all ROS motion processes are stopped and an operator controls power.
2. Apply both patches to disposable copies first and run Python syntax/tests.
3. Build only the affected packages and inspect the installed files.
4. Start with the emergency stop engaged and wheels lifted.
5. Verify exactly one publisher reaches `/cmd_vel`, feedback stops when the
   serial input is interrupted, driver restart remains stopped, and shutdown
   produces repeated zero commands.
6. Re-record low-speed +90/-90 degree tests before enabling navigation.

The RDX packages remain optional experiments. Their safety-topic remap is not part
of the official-course profile and is not needed to reproduce Task 2.
