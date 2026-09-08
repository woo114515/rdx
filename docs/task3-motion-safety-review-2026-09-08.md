# Task 3 motion-logic safety review

Review date: 2026-09-08. Scope: viewpoint recovery and the gated approach,
contact, push, release, return, and inventory cycle. This is a source review,
not a real-robot motion acceptance.

## Implemented in source

### Execute the checked reobservation path

The reobservation node formerly validated a `ComputePathToPose` result against
the temporary cylinder envelope, then sent only its endpoint to
`NavigateToPose`. Nav2 was therefore free to execute a different route. The
node now sends the exact checked `nav_msgs/Path` to `FollowPath`. Its action
parameter is now `follow_path_action: /follow_path`. While following it, the
node also checks the live robot pose and cancels if tracking error carries the
robot centre inside the temporary cylinder clearance envelope.

### Validate reverse retreat in the initial robot frame

The release stage formerly used Euclidean displacement from its origin. A
sideways displacement, heading change, or localization jump could therefore be
counted as the required 0.15 m retreat. It now stores the complete initial pose
and computes rearward progress, lateral drift, and normalized heading error.

Only progress along the initial rearward axis completes release. Forward
displacement beyond 0.02 m, lateral drift above 0.05 m, or heading change above
0.15 rad stops the stage. A translation jump above 0.05 m between control
cycles is also rejected instead of being counted as retreat. These limits are
parameters and still require low-speed real-robot validation.

Neither source change was deployed to the robot as part of this edit.

## Open findings intentionally not changed

The following findings are recorded for later work and must not be interpreted
as implemented guarantees.

1. Target presence currently means any valid LiDAR return in a narrow forward
   corridor. It neither identifies the selected candidate nor handles the
   measured MS200 near blind zone. At the pushing geometry, the cylinder centre
   is about 0.15 m from LiDAR and its near surface is about 0.115 m away, below
   the reported 0.15 m `range_min`.
2. The unexpected-front-obstacle check excludes every return in the nominal
   target corridor. A neighboring cylinder or other obstacle in that corridor
   can be mistaken for the expected target.
3. The direct push path is checked against the remaining-cylinder keepout, but
   not against the occupancy map for the complete robot-and-cylinder swept
   volume.
4. `push_complete` proves only that the robot reached its path endpoint. It
   does not prove that the selected cylinder remained engaged or reached its
   destination. Inventory can subsequently be decremented without an
   independent delivery observation.
5. Cancellation sends an asynchronous Nav2 cancel request and a short burst of
   zero-velocity messages on a shared command topic. There is no exclusive
   command owner or latched stop gate preventing another publisher from
   continuing to command motion.
6. The direct push tracker may reduce linear speed to zero while retaining
   angular speed for a large heading error. Rotation while in contact can shed
   the cylinder from the fork. Push-specific curvature and heading limits are
   still required.
7. Sensor freshness is mainly based on callback arrival time. Scan header age,
   timestamp monotonicity, and TF age are not consistently checked, so replayed
   or stale sensor state can appear fresh.
8. Color confidence is normalized vote purity rather than absolute classifier
   confidence. Repeated weak evidence for only one color can therefore report
   confidence near 1.0.
9. A candidate is called visually `visible` from horizontal projection alone.
   Vertical field of view, physical occlusion, and image health are not part of
   that state; repeated missing detections can consequently cause rejection.
10. Validation auto-locks on the first ready output instead of requiring a
    dwell of consecutive ready frames and bounded positional variation.
11. Visual association remains one-to-one. One merged image box cannot assign
    the same color evidence to multiple spatial candidates even when LiDAR
    resolves multiple cylinders.
12. The snapshot lock service counts all stable tracks, whereas publication
    first selects the primary spatial group. The source set used for locking
    can therefore differ from the set shown as ready, and the lock operation
    does not immediately republish a frozen snapshot.
13. A successful Nav2 return action is accepted as `return_complete` without
    independently checking final position and heading against the recorded
    home pose.
14. The multi-service finalization sequence is not transactional and has no
    overall timeout. A mid-sequence failure can leave planning, validation,
    exclusions, and inventory only partially updated.
15. The occupancy-map filter does not enforce a maximum map age. A delayed or
    stalled SLAM map can affect candidate acceptance while `/scan` remains
    current.

## Safety boundary

Before unrestricted full-cycle testing, the target/contact state must account
for the measured LiDAR blind zone, the complete push swept volume must be
checked against the map, delivery must be independently verified, and stop
commands must have exclusive authority. Until then, motion remains explicitly
gated and tests must be staged at low speed with an operator at the power
control.

## Reobservation field result

The bounded reobservation chain was exercised on the robot on 2026-09-08.
Three checked `FollowPath` goals reached their requested viewpoints and the
node stopped at its configured attempt limit. Candidate-set stability and the
accepted-field proximity gate prevented a transient distant return from
inflating the clearance envelope. However, the same physical arrangement
still produced only five of six expected LiDAR candidates after all three
viewpoints. Automatic recovery of every fully occluded cylinder is therefore
an open limitation. Do not lower the configured inventory to five while six
physical cylinders remain: an omitted cylinder would be absent from the push
keepout geometry. Until a broader viewpoint search is implemented, arrange an
initial unobstructed six-cylinder snapshot and inspect it before arming motion.
