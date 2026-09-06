# color_object_sorter

Task 3 perception implementation. The physical targets changed on 2026-09-06
from blue/legacy-green/pink to blue/replacement-green/red. Blue remains valid;
replacement-green and red were recalibrated, deployed, and visually reviewed
without obvious misclassification in the tested scene on 2026-09-06. This
package has no velocity publisher and cannot move the robot.

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

The former pink reliability result is historical and does not apply to the new
red target. New red and replacement-green thresholds were derived from the
2026-09-06 replacement-target dataset and still require live acceptance.

`temporal_object_confirmation` consumes localized observations and publishes
only stable tracks on `/color_sorter/confirmed_objects`. Its one-second window
requires at least six observations, 60% frame detection, 80% color agreement,
50% LiDAR association, bearing standard deviation at most 3 degrees, and range
standard deviation at most 0.10 m. Spatial reassociation allows a camera
`track_id` to change after a brief missed detection.
All candidates and their metrics are published on
`/color_sorter/confirmation_status` for threshold diagnosis; consumers that
can cause motion must use only `/color_sorter/confirmed_objects`.

Overlapping or occluded targets are unsupported. In particular, independent
per-detection fusion can currently assign one LiDAR cluster to more than one
visual detection, while same-color overlap can merge into one image contour.
These cases must not trigger motion; see the Task 3 fusion document for the
required one-to-one association follow-up.

## Detection viewer

Run the repository-provided viewer in a graphical ROS environment:

```bash
ros2 run color_object_sorter color_sorter_debug_viewer
```

It subscribes to `/color_sorter/debug/compressed` with sensor-data QoS, avoiding
the `rqt_image_view` compressed-transport plugin conflict seen in the supplied
VM. Press `q` or Escape in the window to exit. The viewer has no publishers and
cannot command robot motion.

## HSV sampling

The interactive sampler records labelled pixel patches from the same compressed
stream used by the detector. Usage and collection guidance are documented in
`docs/task3-hsv-sampling.md`.
