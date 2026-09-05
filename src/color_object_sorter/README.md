# color_object_sorter

Fresh Task 3 implementation for detecting and tracking blue, green, and pink
objects. The first milestone is perception-only: this package has no velocity
publisher and cannot move the robot.

It subscribes to `/csi/image_raw/compressed` using sensor-data QoS and publishes
`/color_sorter/detections`, `/color_sorter/debug/compressed`, and
`/color_sorter/perception_health`. HSV values were calibrated from labelled
camera samples on 2026-09-05; see
`docs/task3-hsv-calibration-2026-09-05.md`.

## HSV sampling

The interactive sampler records labelled pixel patches from the same compressed
stream used by the detector. Usage and collection guidance are documented in
`docs/task3-hsv-sampling.md`.
