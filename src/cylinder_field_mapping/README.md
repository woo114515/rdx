# Cylinder field mapping

The preferred Task 3 path is now LiDAR-first and snapshot-based. The legacy
`field_mapper` remains available while the new path is validated.

## LiDAR-first snapshot builder

`snapshot_builder` extracts compact cylinder-sized clusters from `/scan`,
transforms them into `map`, rejects clusters attached to large occupied map
components, merges fragments closer than `minimum_object_separation`, and
accumulates only during an explicit observation session. It publishes:

- `/cylinder_snapshot/candidates`
- `/cylinder_snapshot/markers`

With `keep_all_groups_after_delivery=true`, the initial observation still
publishes only the largest connected candidate group. Once the controller
installs at least one delivered-object exclusion zone, every stable spatial
group is published for visual validation. This prevents an isolated final
cylinder from being discarded before its color can be checked.

It never publishes velocity. Start it and begin a fresh observation session:

```bash
ros2 launch cylinder_field_mapping snapshot.launch.py
ros2 service call /cylinder_snapshot/start std_srvs/srv/Trigger '{}'
ros2 topic echo /cylinder_snapshot/candidates
```

When the reported snapshot is ready, freeze it for downstream color assignment:

```bash
ros2 service call /cylinder_snapshot/lock std_srvs/srv/Trigger '{}'
```

After one push, discard all previous positions and start again:

```bash
ros2 service call /cylinder_snapshot/reset std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_snapshot/start std_srvs/srv/Trigger '{}'
```

Object inventory is configuration, not code. `inventory_colors` and
`inventory_counts` are parallel lists. Their sum determines the expected total;
empty lists enable count-agnostic operation. Adding a color requires only a new
inventory entry and a matching HSV range in the perception configuration.

## Legacy visual-first field mapper

This package converts confirmed camera/LiDAR observations into persistent
map-frame cylinder records. It associates observations by physical position,
not by the short-lived camera `track_id`, and publishes the current minimum
enclosing circle of stable cylinder centers.

The node has no velocity publisher and cannot move the robot.

## Topics and services

- subscribes: `/color_sorter/confirmed_objects`
- publishes: `/cylinder_field/objects`
- publishes: `/cylinder_field/markers`
- service: `/cylinder_field/lock_initial_envelope`
- service: `/cylinder_field/reset`

The current envelope changes as the map improves. The locked initial envelope
does not change and will later be used as a keep-out region by the sorting
planner. Its radius includes the configured physical cylinder radius.

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

While a snapshot is collecting, `/cylinder_snapshot/filter_diagnostics`
publishes one JSON record per scan. It reports raw clusters and rejections at
the observation-sector, workspace, delivered-object exclusion, occupancy-map,
nearby-fragment merge, temporal-stability, and primary-spatial-group stages.
Use it to diagnose missing or surplus candidates without changing thresholds:

```bash
ros2 topic echo /cylinder_snapshot/filter_diagnostics --once
```

`use_map_filter` controls whether the accumulated occupancy map may veto a
live scan cluster. Task 3 disables that veto because GMapping can lag a moved
cylinder; the bounded forward sector, compact-cluster geometry, temporal
tracking, spatial grouping, inventory, and visual validation remain active.
`require_map_filter` only controls behavior when map filtering is enabled but
no map has arrived.
