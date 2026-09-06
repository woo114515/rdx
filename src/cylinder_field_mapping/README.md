# Cylinder field mapping

This package converts confirmed camera/LiDAR observations into persistent
map-frame cylinder records. It associates observations by physical position,
not by the short-lived camera `track_id`, and publishes the current minimum
enclosing circle of stable cylinder centers.

The node has no velocity publisher and cannot move the robot.

## Topics and services

- subscribes: `/color_sorter/confirmation_status`
- publishes: `/cylinder_field/objects`
- publishes: `/cylinder_field/markers`
- service: `/cylinder_field/lock_initial_envelope`
- service: `/cylinder_field/reset`

The current envelope changes as the map improves. The locked initial envelope
does not change and will later be used as a keep-out region by the sorting
planner. Its radius includes the configured physical cylinder radius.

The current topic and RViz markers contain confirmed and temporarily occluded
objects. A reliable measurement refreshes position. Ambiguous, tentative, or
missing measurements never overwrite the last reliable position. After
`occluded_after`, the marker becomes translucent and reports `occluded`; after
`stale_after`, it leaves the active map. The locked initial envelope remains
unchanged.

Normal mapping never relocates a target across the field. Association is
color-gated and each configured color is limited to two landmarks. Deliberate
motion will require a future explicit `moving` state owned by the sorter.

Start after perception and the `map -> lidar_link` TF chain are available:

```bash
ros2 launch cylinder_field_mapping field_mapping.launch.py
ros2 topic echo /cylinder_field/objects
```

After all six cylinders have at least three observations, lock the initial
group boundary:

```bash
ros2 service call /cylinder_field/lock_initial_envelope std_srvs/srv/Trigger '{}'
```

Overlapping same-color cylinders that appear as one camera contour cannot
produce two trustworthy ranges. The upstream perception pipeline marks an
ambiguous association as unconfirmed, so it is intentionally excluded here.
The robot must acquire another viewpoint to separate those targets.
