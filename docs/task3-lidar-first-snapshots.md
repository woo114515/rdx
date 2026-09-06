# Task 3 LiDAR-first snapshot design

## Decision

Spatial object existence and position come from LiDAR. Vision supplies color
evidence after spatial candidates exist. A snapshot is rebuilt before every
push and discarded after that push; the system does not maintain a permanent
six-object dynamic map.

## Configurable inventory

No algorithm assumes six objects or three fixed color names. The task inventory
is represented by two validated parameters:

```yaml
inventory_colors: [blue, green, red]
inventory_counts: [2, 2, 2]
```

The expected total is derived from the counts. Empty lists mean that only
`minimum_object_count` applies. New colors can be introduced by adding an
inventory entry and the corresponding HSV ranges. Future global color
assignment will consume the same inventory.

## Current implementation boundary

Implemented in this increment:

- generic compact-cluster extraction from `LaserScan`;
- rejection of candidates attached to large `/map` occupancy components;
- acceptance of live LiDAR candidates that are absent from the accumulated
  map, because movable cylinders may have changed position without triggering
  a Gmapping update;
- expiration of candidate tracks after six consecutive unmatched scans, which
  prevents stale IDs and replacement IDs for one physical cylinder from being
  published together while preserving brief scan dropouts;
- merging of fragments closer than the configured minimum object separation;
- map-frame multi-scan accumulation;
- configurable workspace bounds;
- explicit start, lock, and reset services;
- candidate/status message and RViz markers;
- count validation derived from configurable inventory;
- configurable forward observation sector in the live LiDAR frame;
- projection of map-frame candidates into the current camera view;
- multi-frame color evidence with validated, uncertain, unobserved, and rejected
  states;
- inventory validation for arbitrary configured color names and counts;
- pure Python tests with no connected robot.

Not implemented yet:

- circle-model scoring against recorded MS200 data;
- optimal global color assignment when competing evidence cannot be resolved by
  the current per-candidate vote;
- active viewpoint changes;
- approach, navigation, or pushing.

The next gate is a stationary real-world recording. The candidate extractor must
reliably reproduce the visible physical objects before any motion feature is
connected.

## Observation semantics

The initial target group is assumed to be in front of the robot, but individual
objects may be outside the camera image. The LiDAR snapshot therefore limits new
candidates by configurable range, bearing, and lateral bounds. Visual validation
never treats an out-of-frame candidate as background:

- `validated`: enough consistent color evidence;
- `rejected`: repeatedly visible with no configured target color;
- `unobserved`: outside the image or behind the camera;
- `uncertain`: visible but evidence is not yet sufficient.

The LiDAR snapshot may be locked with extra candidates so the visual stage can
remove clutter. The validated snapshot can only be locked when its color counts
match the configured inventory.

## Stationary acceptance result — 2026-09-07

The LiDAR-first candidate path passed its first stationary field acceptance.
Six cylinders were physically present, but one was fully occluded from the
single 2D LiDAR view. RViz showed five corresponding spatial candidates and the
node reported exactly:

```text
expected_object_count: 6
ready: false
status: incomplete:5/6
```

All five retained candidates reached the 30-observation history limit. The
previous wall/boundary explosion (`ambiguous:70/6`, then `ambiguous:17/6`) was
removed by map-component filtering, close-fragment merging, the initial forward
sector, and primary spatial-group selection. Most importantly, the missing
physical return was not replaced by a wall candidate merely to satisfy the
configured inventory. This is the required failure behavior for a fully
occluded target.

The visual validation output contained five spatial candidates:

```text
status: validated:4;uncertain:1;unobserved:0;rejected:0
ready: false
```

Two candidates were validated as blue, one as red, and one as green. The fifth
candidate received only two blue observations out of 151 visible frames and
correctly remained `uncertain`. The spatial snapshot behavior is accepted. Color
classification is not yet accepted as fully reliable and is deliberately left
unchanged for now; no HSV thresholds or validation parameters were adjusted from
this result.

The currently deployed visual association is also intentionally provisional:
one visual detection is allocated to at most one LiDAR candidate. That prevents
a wall candidate on the same bearing from borrowing a cylinder's color, but it
cannot correctly represent two distinct same-color cylinders merged into one
wide image region. The planned replacement is per-candidate image sampling with
same-bearing depth/occlusion handling. Until that work is completed and the
configured color inventory is satisfied, `ready: false` must block snapshot
locking and all pushing decisions.

## LiDAR and color-validation stage closure — 2026-09-07

Later stationary testing with all six cylinders visible produced six correctly
colored validated targets (two blue, two green, and two red). A seventh compact
LiDAR return was retained as a raw candidate but accumulated no color evidence
and was correctly rejected by visual validation, giving `ready: true` at the
validated inventory layer. Raw LiDAR candidate count may therefore exceed the
configured inventory; downstream decisions must consume the locked validated
snapshot rather than assuming every compact return is a target.

Field testing also exposed and corrected three failure modes: movable cylinders
were rejected when stale Gmapping data lacked occupied cells at their new
positions; the linear camera/LiDAR projection was too strict near image edges;
and stale candidate tracks accumulated replacement IDs. The map now acts only
as a veto for known large connected obstacles, edge associations use a wider
one-to-one matching window, and unmatched tracks expire after six scans.

Candidate IDs are deliberately snapshot-local. They remain stable for the
current collection and lock cycle, but are discarded after reset and must not
be treated as permanent physical-object identities. This stationary perception
stage is closed; the next stage is motion-free selection and visualization of a
target, approach pose, envelope exit curve, and destination push path.

The validator now locks automatically on the first frame whose validated color
counts exactly match the configured inventory. This intentionally freezes the
earliest complete snapshot before longer-running SLAM drift can create
replacement candidate IDs. `/cylinder_validation/lock` remains available for
manual operation when `auto_lock_when_ready` is disabled. Reset both the LiDAR
snapshot and validator before collecting the next post-push snapshot. RViz and
planning should consume `/cylinder_snapshot/validation_markers`; raw candidate
markers remain diagnostic and may include visually rejected clutter.

The locked validation output now uses reliable, transient-local QoS. This fixes
the startup race in which the validator published its only locked sample before
the selection planner or a command-line listener had subscribed. The complete
motion-free pipeline can be started with:

```bash
ros2 launch cylinder_push_planner task3_planning.launch.py
```

It starts the snapshot builder, color detector, candidate validator, and
selection planner. Camera, LiDAR, mapping, TF, and base drivers remain external
prerequisites. The combined launch must not be run together with the individual
snapshot or LiDAR-first validation launches, because duplicate node instances
would process and publish the same topics.
