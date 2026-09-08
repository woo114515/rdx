# cylinder_push_planner

This package converts a locked, validated cylinder snapshot into planning
geometry and provides a separately gated execution node. The planner itself
does not publish `Twist` or invoke Nav2; execution is disabled by default.

Start perception, planning, and the default-disabled execution node with:

```bash
ros2 launch cylinder_push_planner task3_planning.launch.py
ros2 service call /cylinder_snapshot/start std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_plan/generate std_srvs/srv/Trigger '{}'
```

This default command cannot initiate motion. The execution node becomes
motion-capable only with `enable_push_cycle_motion:=true`, and every motion
stage still needs its own arm/start service pair.

The launch starts the selection planner, bounded reobservation controller,
push-cycle executor, LiDAR snapshot builder, color detector, and candidate
validator. Do not also
start `snapshot.launch.py` or
`lidar_first_validation.launch.py`, because that would create duplicate nodes.
The camera, LiDAR, TF, Nav2, and mapping/base launch remain external hardware
prerequisites.

The locked `/cylinder_snapshot/validated_objects` result uses reliable,
transient-local QoS. A planner or diagnostic subscriber that starts after the
one-shot automatic lock therefore still receives the latest result.

RViz can display `/cylinder_push_plan/markers`. The preview contains:

- the orange initial all-cylinder envelope;
- the red selected right-side target and blue staging arrow;
- the purple convex hull of every *unselected* cylinder;
- the red expanded robot-centre keepout around that hull;
- the yellow approach trajectory, green target trajectory, cyan robot pushing
  trajectory, and white release/return trajectory;
- the green destination disk and white task-start pose.

After a delivery is explicitly finalized, its planned destination is added to
a bounded LiDAR exclusion list before the next reduced-inventory snapshot is
started. This prevents a cylinder already placed in a sorting area from being
counted again. The planner rejects a destination whose exclusion disk would
overlap a cylinder that remains in the field.

The selected target is removed before the remaining-cylinder convex hull is
built. Both the sampled robot pushing trajectory and return trajectory must stay
outside its configured expansion. The cylinder trajectory receives a separate
clearance check, and the destination must be outside the initial all-cylinder
envelope. Destinations are configured by color in `config/selection.yaml` as
coordinates relative to the robot pose captured at generation time (`+x`
forward, `+y` left).

Each color coordinate is the centre of a sorting zone, not a single shared
drop point. The planner learns the initial count for every configured color
from the first inventory message and allocates successive tangential slots at
the configured spacing. This supports arbitrary colors and counts and prevents
the second same-color cylinder from being pushed directly into the first.
The robot pose captured for the first generated plan is retained as the task
home anchor. Later sorting-zone coordinates and return goals use that fixed
anchor, while the live pose is used only as the next approach path's start.
This prevents return error or localization drift from moving every destination
on successive cycles.

`/cylinder_push_plan/reset` clears only the current preview and deliberately
preserves that anchor and slot history for the next push. Before starting a
completely new field, call `/cylinder_push_plan/reset_task`; it clears both.
The snapshot builder separately owns delivered-object exclusions and the
remaining inventory, so a completely new field must reset those explicitly:

```bash
ros2 service call /cylinder_validation/reset std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_snapshot/reset std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_snapshot/set_exclusions \
  color_object_sorter_interfaces/srv/SetSnapshotExclusions \
  '{centers: [], radius: 0.0}'
ros2 service call /cylinder_snapshot/set_inventory \
  color_object_sorter_interfaces/srv/SetObjectInventory \
  '{colors: [blue, green, red], counts: [2, 2, 2]}'
ros2 service call /cylinder_push_plan/reset_task std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_execution/reset std_srvs/srv/Trigger '{}'
```

Use the inventory configured for the new field; the shown `2/2/2` values are
only the current competition setup. These services do not command motion.

The curve search evaluates both directions over several bend angles and radii.
It chooses the shortest candidate that passes every clearance test, allowing a
target assigned to the opposite side of the field to take a wider route around
the remaining hull without weakening the configured safety distance.

This is still a motion-free geometric preview. It does not inspect occupancy
grid collisions, call Nav2, control contact, verify that the cylinder remains in
the fork, or publish velocity. A successful `generate` response therefore does
not authorize real movement.

## Gated push-cycle execution

`approach_execution` provides independently gated approach, push, release and
return stages. It is disabled by default. When explicitly enabled, it still
requires a fresh locked snapshot and preview, live
LiDAR and TF, zero Nav2 lateral-velocity limits, and a two-command arm/start
sequence. `ComputePathToPose` first produces a costmap-checked path; that exact
path is rejected if it enters the expanded remaining-cylinder polygon, then
sent to Nav2 `FollowPath` so the controller cannot silently replan through the
polygon.

The arm expires after ten seconds. Cancellation, stale LiDAR, lost TF, timeout,
Nav2 rejection, polygon entry, and shutdown cancel the action and publish a
zero-velocity burst. A successful approach ends at a Nav2-safe pre-contact
pose. Separate contact arm/start calls then creep the final 0.25 m and stop
again at `contact_complete`; pushing still requires its own arm/start calls.
Direct pushing is forward-only and capped at 0.06 m/s, tracks the sampled robot
path, requires the target to remain in a narrow LiDAR corridor, and stops for
any close return
outside that corridor. `arm_return` and `start_return` perform a monitored
0.15 m reverse release before asking Nav2 for a fresh collision-checked path
home. Release completion uses displacement projected onto the robot's initial
rearward axis; excessive lateral drift or heading change stops the manoeuvre.
Finally, `finalize_delivery` resets perception, decrements the delivered
color count, and starts the reduced-inventory snapshot. Do not enable any
motion stage until its RViz paths have been reviewed and the physical test area
is clear.

The intended real-robot sequence is deliberately manual at every boundary:

```bash
# Only after the static preview has been reviewed and the operator is at power:
ros2 launch cylinder_push_planner task3_planning.launch.py \
  enable_push_cycle_motion:=true

ros2 service call /cylinder_push_execution/arm std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_execution/start_approach std_srvs/srv/Trigger '{}'
# Wait for state=approach_complete, inspect alignment, then creep to contact:
ros2 service call /cylinder_push_execution/arm_contact std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_execution/start_contact std_srvs/srv/Trigger '{}'
# Wait for state=contact_complete, inspect fork contact, then:
ros2 service call /cylinder_push_execution/arm_push std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_execution/start_push std_srvs/srv/Trigger '{}'
# Wait for state=push_complete and inspect the delivered cylinder, then:
ros2 service call /cylinder_push_execution/arm_return std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_execution/start_return std_srvs/srv/Trigger '{}'
# Wait for state=return_complete, then start the reduced snapshot:
ros2 service call /cylinder_push_execution/finalize_delivery \
  std_srvs/srv/Trigger '{}'
```

Monitor `/cylinder_push_execution/status` throughout. At any point, call
`/cylinder_push_execution/cancel`; Ctrl+C and detected faults also request a
zero-velocity burst. This is the source-level first implementation. It must not
be treated as competition-ready until each stage has passed low-speed physical
tests independently.

## Inventory transition between pushes

The initial inventory is configured in
`cylinder_field_mapping/config/snapshot.yaml`. After one cylinder has actually
been delivered, the snapshot builder can accept reduced counts through
`/cylinder_snapshot/set_inventory`. The service is rejected while a snapshot is
collecting or locked, so evidence must be reset before changing it. For example,
after delivering one blue cylinder:

```bash
ros2 service call /cylinder_validation/reset std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_snapshot/reset std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_snapshot/set_inventory \
  color_object_sorter_interfaces/srv/SetObjectInventory \
  "{colors: [blue, green, red], counts: [1, 2, 2]}"
ros2 service call /cylinder_snapshot/start std_srvs/srv/Trigger '{}'
```

This interface never commands motion. The execution controller must derive the
new counts from the locked snapshot and delivered target rather than hard-code
six objects.

## Occlusion recovery

`cylinder_reobservation` handles an incomplete stationary snapshot. It derives
left and right poses on a safe circle around the observed group, asks Nav2 to
compute both paths, rejects paths that enter the group clearance circle or
exceed configured distance limits, and prefers the right path when similarly
safe. Execution sends that exact checked path to Nav2 `FollowPath`; it does not
send only the endpoint for replanning. After an executed viewpoint change it
waits for sensor stabilization,
resets both evidence stores, and starts a new stationary collection. Attempts
and total travel are bounded. The clearance circle includes a configurable
30 cm allowance for the unseen cylinder, not only the envelope of the five
visible returns.

Robot testing confirmed execution of three checked viewpoint paths, but one
fully occluded arrangement remained at five of six candidates after the
configured attempt limit. This recovery is not guaranteed yet. Never reduce
the inventory merely to make such a snapshot report ready; reposition the
field or robot until all physical cylinders are represented before generating
a push plan.

The combined launch defaults to preview mode and cannot send a navigation goal:

```bash
ros2 launch cylinder_push_planner task3_planning.launch.py
ros2 service call /cylinder_reobservation/start std_srvs/srv/Trigger '{}'
```

Display `/cylinder_reobservation/markers` and
`/cylinder_reobservation/path` in RViz. Inspect the retained status with:

```bash
ros2 topic echo /cylinder_reobservation/status --once \
  --qos-reliability reliable --qos-durability transient_local
```

Only after the preview is accepted, restart the launch with explicit execution:

```bash
ros2 launch cylinder_push_planner task3_planning.launch.py \
  enable_reobservation_motion:=true
ros2 service call /cylinder_reobservation/start std_srvs/srv/Trigger '{}'
```

Cancel at any time with `/cylinder_reobservation/cancel`. Cancellation, stale
LiDAR, lost TF, timeout, action failure, and node shutdown cancel the Nav2 goal
and publish a burst of zero `Twist` messages. The controller never publishes a
non-zero `Twist`; motion is delegated to Nav2. The deployed Nav2 controller must
keep `min_vel_y` and `max_vel_y` at zero so the mecanum base does not strafe.
