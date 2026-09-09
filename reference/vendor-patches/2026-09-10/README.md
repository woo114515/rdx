# Task 2 Nav2 goal-checker regression fix

`yahboomcar-nav-task2-goal-checker-fix.patch` restores the shared vendor DWB
configuration to one registered goal checker. Nav2 Humble's default
`NavigateToPose` behavior tree sends `FollowPath` without a `goal_checker_id`.
When two checkers were registered, `controller_server` rejected the empty name
and aborted every Task 2 navigation goal.

The `task3_goal_checker` parameter block may remain in the YAML, but it is not
registered by `goal_checker_plugins`. The accepted Task 3 implementation in
`simple_task3` publishes `/cmd_vel` directly and does not start DWB or use this
vendor parameter file. The legacy `cylinder_push_planner` Nav2 execution path
does explicitly request `task3_goal_checker` and must not be used with this
shared Task 2 configuration.

Robot backup made before deployment:

`/home/sunrise/vendor-backups/2026-09-10-before-task2-goal-checker-fix/dwb_nav_params.yaml`

Restart `navigation_dwb_launch.py` after applying the patch because controller
plugins are loaded when Nav2 is configured.

## Bottom-controller serial write recovery

`yahboomcar-driver-write-recovery.patch` changes runtime motion and watchdog
zero writes from fatal exceptions to throttled ROS ERROR reports. A transient
write failure therefore leaves `Mcnamu_driver` alive and the following command
or watchdog tick can retry. Successful writes reset the consecutive failure
counter. Startup still fails if the driver cannot send its initial repeated
zero command, and shutdown still attempts five zero commands.

The ERROR includes the operation, consecutive failure count, receive-thread
state, latest-byte age, and the low-level serial error retained by
`SunriseRobotLib`. This fixes the previous loss of the actual `OSError`, serial
exception, short-write, or timeout reason behind the generic process crash.

Robot backup made before deployment:

`/home/sunrise/vendor-backups/2026-09-10-before-driver-write-recovery/`
