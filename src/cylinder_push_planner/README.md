# cylinder_push_planner

## Compact direct architecture (current Task 3 path)

`task3_compact.launch.py` is the reduced runtime architecture. It preserves
the already validated six-cylinder perception chain unchanged and replaces the
selection planner, reobservation controller, staged Nav2 executor, and service
orchestrator with one `direct_task_controller` process. The launch therefore
starts four project nodes rather than seven:

- `cylinder_snapshot_builder`;
- `color_object_detector`;
- `cylinder_candidate_validator`;
- `cylinder_direct_task_controller`.

The direct controller does not call Nav2 and does not dynamically replan. It
selects an outside target, removes that target from the remaining-cylinder
set, expands the remaining convex hull by the configured robot-centre
clearance, and constructs approach, push, and return curves outside that
keepout. The cycle is rejected only if no valid geometric curve can be found.
Live scan, TF,
target-presence, path-deviation, timeout, cancel, and zero-command shutdown
checks remain active. Motion is disabled unless explicitly enabled:

Before entering the short contact segment, the compact controller now stops
and reacquires the selected target from three stable fresh near-range LiDAR
scans. It compares cylinder-sized clusters with the selected target's predicted
2D position, rejects an ambiguous neighbour, transforms the accepted centre
into `odom`, and rebuilds the remaining approach/contact/push/return geometry.
Ambiguous or split-cluster scans are skipped rather than assigned. This
corrects the lateral dead-reckoning error accumulated during the long approach
without allowing SLAM corrections to move the active route.

The measured base motor threshold is approximately `0.15 m/s`. The compact
controller therefore clamps every non-zero linear command to at least that
value, while all normal stops and safety shutdowns still publish exact zero.
After subsequent path-tracking tests, all compact-controller translation
phases were restored to the measured minimum reliable speed of `0.15 m/s`.
The angular gain and angular-speed limit remain `3.0` and `0.50 rad/s`.

For the current controlled Task 3 field, `live_obstacle_stop_enabled` is false:
the controller follows corridors checked against the locked cylinder snapshot
without stopping on later front/rear scan returns or attempting avoidance.
Scan freshness and a pre-contact selected-target confirmation are still
required. After contact begins, near-range LiDAR target-loss does not stop the
locked push path because the mechanical fork retains the cylinder. TF loss,
path deviation, timeouts, cancellation, and shutdown still command zero. Task 2
Nav2 obstacle handling is not changed.

Targets, remaining-cylinder keepouts, generated paths, the saved home pose, and
the live tracking pose all use `map`. A GMapping correction therefore changes
the robot pose used by the controller without rotating a previously screened
route away from the mapped cylinders. Physical-motion continuity and the
15 cm release retreat are checked separately in stamped `/odom`, where a map
correction cannot look like wheel travel.

The final in-place rotation returns to the `map -> base_footprint` yaw captured
at task start. A five-sample circular window must be stable before the heading
can be accepted. `/imu/data_raw` is retained only to require a settled physical
turn rate; its full-task yaw integral is diagnostic and no longer decides
completion. Missing fixed-frame TF, IMU loss, or an excessive IMU gap stops
motion. This does not modify the vendor EKF used by Task 2.

`release_forward_motion_guard_enabled` controls the release-stage forward
progress abort and is false in the current Task 3 configuration. This avoids a
false stop from a short odometry correction while switching to reverse. Reverse
completion, lateral and heading drift, timeout, odometry, TF, path-deviation,
operator cancellation, and zero-command shutdown checks remain active.

`return_path_deviation_guard_enabled` is false for the current retraced return.
The return path reverses the screened push, contact, and approach corridors;
it does not generate a new shortest delivery-to-home chord. The robot continues
correcting toward each return waypoint instead of aborting solely because
accumulated base or odometry error exceeded 0.12 m. The common path-deviation
guard remains active during approach, contact, and push.

Contact-time target reacquisition is intentionally a local correction. The
initial approach, target push curve, and return corridors retain their convex
remaining-cylinder keepout checks. If a fresh `map -> odom` correction makes
the already reached staging pose appear inside that convex hull, only the new
short staging-to-staging approach may use per-cylinder clearance instead. It
must remain at least `transit_clearance + cylinder_radius` from every
non-selected cylinder. The corrected return retraces this local segment and
then the original approach that the robot already traversed successfully.

```bash
ros2 launch cylinder_push_planner task3_compact.launch.py \
  enable_motion:=true

# Reset perception and begin collection for the configured inventory.
ros2 service call /cylinder_task/prepare std_srvs/srv/Trigger '{}'

# After validated_objects reports ready=true and locked=true, one call plans,
# approaches, contacts, pushes, releases, returns, and starts the next snapshot.
ros2 service call /cylinder_task/run_once std_srvs/srv/Trigger '{}'

# Or replace prepare + repeated run_once calls with one full-inventory command.
# It prepares six cylinders, pushes one at a time, returns home after each one,
# rebuilds the remaining snapshot, and stops only at task_complete or a fault.
ros2 service call /cylinder_task/run_all std_srvs/srv/Trigger '{}'

# Available throughout motion.
ros2 service call /cylinder_task/cancel std_srvs/srv/Trigger '{}'
```

The previous `task3_planning.launch.py` and its low-level services remain in
the package as a rollback and diagnostic path. Do not run the compact and
legacy launches together because their task services and velocity publishers
conflict.

`run_all` returns immediately after accepting the operation; completion is
reported asynchronously on `/cylinder_task/status`. It includes the initial
`prepare`, so do not call `prepare` first. Each reduced-inventory snapshot must
reach `ready=true` and `locked=true` within 120 seconds. A perception timeout or
any motion fault cancels the automatic loop and publishes zero velocity; it does
not retry a failed motion stage.

## Legacy staged architecture

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

The combined launch selects Fast DDS `UDPv4` for all child processes to avoid
the shared-memory port-lock failure observed on the robot.  Before issuing
direct `ros2 service`, `topic`, or `param` commands in another terminal, source
the deployed helper once:

```bash
source /home/sunrise/yahboomcar_ws/task3_ros_env.sh
```

This transport override is deliberately scoped to Task 3; see
`docs/fastdds-udp-transport.md` for the diagnosis and rollback option.

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
offsets from the first six-cylinder envelope centre. Their axes use the robot
heading captured at first generation (`+x` forward, `+y` left).

Each color coordinate is the centre of a sorting zone, not a single shared
drop point. The planner learns the initial count for every configured color
from the first inventory message and allocates successive tangential slots at
the configured spacing. This supports arbitrary colors and counts and prevents
the second same-color cylinder from being pushed directly into the first.
The robot pose captured for the first generated plan is retained as the task
home anchor. The initial field centre is retained separately as the sorting-zone
origin. Later sorting-zone coordinates use that fixed field origin, return
goals use the fixed home anchor, and the live pose is used only as the next
approach path's start. This prevents return error, localization drift, or a
shrinking remaining-cylinder envelope from moving destinations between cycles.

Perception snapshots remain expressed in `map`. The approach, contact, push,
return, remaining-cylinder keepout, and delivered-destination exclusion
geometry remains in that same frame throughout a compact cycle. `/odom` is not
used to express obstacle geometry or long paths; it is used only for physical
motion continuity and release-retreat distance.

Each new collection also establishes an inventory and timestamp generation.
The controller clears its cached snapshot and accepts only a locked result with
the exact remaining color counts and a header timestamp after that generation
started. A transient-local sample from the preceding cycle can no longer start
the next motion cycle.

`/cylinder_push_plan/reset` clears only the current preview and deliberately
preserves those anchors and slot history for the next push. Before starting a
completely new field, call `/cylinder_push_plan/reset_task`; it clears them.
The executor similarly exposes `/cylinder_push_execution/reset_task` to clear
its delivered-destination history; ordinary execution `reset` deliberately
preserves that history between pushes. The snapshot builder separately owns
delivered-object exclusions and the remaining inventory, so a completely new
field must reset those explicitly:

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
ros2 service call /cylinder_push_execution/reset_task std_srvs/srv/Trigger '{}'
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

### Bundled task controls

`task3_planning.launch.py` also starts a thin orchestration node. It retains all
low-level services for diagnosis. Normal operation uses preparation, planning,
and one automatic execution call:

```bash
# New field: reset execution, plan, validation and snapshot state; clear old
# exclusions; load the configured inventory; and begin stationary collection.
ros2 service call /cylinder_task/prepare std_srvs/srv/Trigger '{}'

# After validated_objects reports ready=true and locked=true, call once. It
# generates a fresh plan and then automatically performs approach, contact,
# push, release, return and delivery finalization.
ros2 service call /cylinder_task/run_once std_srvs/srv/Trigger '{}'

# Available during execution:
ros2 service call /cylinder_task/cancel std_srvs/srv/Trigger '{}'
```

`run_once` first regenerates the preview and waits until every stamped path
component has reached the executor. It then follows the executor's reported
state instead of using fixed delays. All existing input freshness, collision,
target-presence, timeout and zero-velocity fault checks remain active. A failure
cancels the cycle rather than retrying motion. `/cylinder_task/generate` and
`/cylinder_task/advance` remain available for staged
diagnosis, but is not part of normal operation. Inspect `/cylinder_task/status`
and `/cylinder_push_execution/status` while the cycle runs. Inventory is configured in
`config/task_orchestration.yaml`; colors and counts remain aligned lists so the
workflow is not tied to six cylinders or three colors.

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
