# color_object_sorter

Task 3 perception implementation. The current physical targets are blue, a new
green, and red. The pink, earlier green, and black targets are retired. The new
green was calibrated from a dedicated green/background dataset, while the
previously accepted blue and red ranges remain in use. This package has no
velocity publisher and cannot move the robot.

The configured color names are not fixed in the detector algorithm. To add a
new color, append its name to `colors` in `config/perception.yaml`, then add
`hsv.<name>.range_count` and the corresponding `lower_N`/`upper_N` entries.
Unknown names are declared from YAML at startup; no Python source edit is
required. The task inventory and per-color quantities are maintained separately
in `cylinder_field_mapping/config/snapshot.yaml`.

It subscribes to `/csi/image_raw/compressed` using sensor-data QoS and publishes
`/color_sorter/detections`, `/color_sorter/debug/compressed`, and
`/color_sorter/perception_health`. HSV values were calibrated from labelled
camera samples on 2026-09-05; see
`docs/task3-hsv-calibration-2026-09-05.md`.

`color_lidar_fusion` associates detections with coherent `/scan` clusters and
publishes `/color_sorter/localized_objects`. The initial stationary calibration
maps image center `normalized_x=-0.069` to LiDAR 0 rad and
`normalized_x=-0.736` to approximately +15 degrees. Matches are rejected when
the scan is stale or no multi-beam cluster exists. Neither perception node can
publish velocity commands.

The former pink, earlier-green, and black reliability results are historical
and do not apply to the current target inventory. See
`docs/task3-hsv-sampling.md` for the current calibration evidence.

`temporal_object_confirmation` consumes localized observations and publishes
only stable tracks on `/color_sorter/confirmed_objects`. Its one-second window
requires at least six observations, 60% frame detection, 80% color agreement,
50% LiDAR association, bearing standard deviation at most 3 degrees, and range
standard deviation at most 0.10 m. Spatial reassociation allows a camera
`track_id` to change after a brief missed detection.
All candidates and their metrics are published on
`/color_sorter/confirmation_status` for threshold diagnosis; consumers that
can cause motion must use only `/color_sorter/confirmed_objects`.

The newer LiDAR-first path uses `candidate_validator`. It consumes spatial
candidates from `/cylinder_snapshot/candidates` and image detections, then
publishes `/cylinder_snapshot/validated_objects` plus RViz markers on
`/cylinder_snapshot/validation_markers`. Candidates outside the image remain
`unobserved`; only repeatedly visible candidates without configured color
evidence become `rejected`. Lock a complete validated inventory with
`/cylinder_validation/lock`, or clear all accumulated evidence with
`/cylinder_validation/reset`.

The validated result and validation markers use reliable, transient-local QoS.
The final automatically locked sample is retained for late-starting planning
and RViz subscribers instead of disappearing after its one publication.

Fusion now extracts LiDAR clusters once per scan and assigns them globally, so
one cluster cannot be reused by two visual detections. Competing detections and
a wide merged contour spanning similarly plausible clusters are marked
`ambiguous` and cannot be confirmed. A temporarily missing temporal track is
reported as `occluded`. Same-color image contours can still merge, and a fully
hidden object cannot be recovered from a single 2D scan; these cases require a
new viewing position and must not trigger motion.

The candidate validator supports a lightweight range-aware camera/LiDAR
projection model. The accepted legacy two-point model remains the default until
an operator captures a stationary multi-angle, multi-range dataset and enables
the fitted model explicitly. Capture, fitting, validation criteria, and the
projection residual diagnostics topic are documented in
`docs/task3-camera-lidar-calibration.md`.

## Detection viewer

Run the repository-provided viewer in a graphical ROS environment:

```bash
ros2 run color_object_sorter color_sorter_debug_viewer
```

It subscribes to `/color_sorter/debug/compressed` with sensor-data QoS, avoiding
the `rqt_image_view` compressed-transport plugin conflict seen in the supplied
VM. Press `q` or Escape in the window to exit. The viewer has no publishers and
cannot command robot motion.

## Six-cylinder RViz view

Open the preconfigured lightweight map view with:

```bash
ros2 launch color_object_sorter task3_visualization.launch.py
```

It fixes the frame to `map` and loads only the occupancy map, the current laser
scan, and `/cylinder_snapshot/validation_markers`. The scan has zero decay, the
subscriber queues have depth one, and RViz renders at 10 FPS to reduce long-run
network and rendering pressure. Raw candidate markers and navigation costmaps
are intentionally omitted; the colored markers are the validated six-cylinder
result.

## HSV sampling

The interactive sampler records labelled pixel patches from the same compressed
stream used by the detector. Usage and collection guidance are documented in
`docs/task3-hsv-sampling.md`.
