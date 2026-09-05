# Smooth figure-eight task

`rdx_base_control` follows a sampled Gerono lemniscate with pure pursuit. It
uses forward velocity and yaw only, so it avoids the unreliable mecanum strafe.
Progress is monotonic to prevent switching lobes at the centre crossing.

The controller publishes `/cmd_vel_nav`; `rdx_safety` remains the only outlet
to `/cmd_vel`. The requested limits are 0.66 m/s linear and 1.0 rad/s angular,
with slew-rate limiting. The safety node's lower configured limits still win.

The robot must have the base driver, odometry TF and safety node running. Start
the controller first; it publishes zero until explicitly armed:

```bash
source /opt/tros/humble/setup.bash
source ~/rdx/install/setup.bash
ros2 run rdx_base_control figure_eight
```

After clearing the area and placing an operator by power control:

```bash
ros2 service call /rdx_figure_eight/start std_srvs/srv/Trigger '{}'
```

Cancel with:

```bash
ros2 service call /rdx_figure_eight/cancel std_srvs/srv/Trigger '{}'
```

Missing TF, excessive path error, timeout, cancel and shutdown all command zero.
