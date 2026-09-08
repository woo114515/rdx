# Task 3 Camera/LiDAR Calibration

The Task 3 validator is LiDAR-first: the laser supplies candidate range and
bearing, while the camera supplies color. The original two-point linear model
is retained as `projection_model: legacy_linear` until a multi-point stationary
calibration is collected and accepted. None of the tools in this document
publishes a velocity command.

## Why calibration is required

The legacy mapping cannot represent lens-edge distortion or parallax caused by
the camera and LiDAR having different optical origins. The lightweight
replacement predicts normalized image x from LiDAR bearing and range using six
terms: constant, angle, angle squared, angle cubed, inverse range, and angle
times inverse range. Runtime cost is negligible for six candidates.

## Collect correspondences

Use one clearly detected cylinder, keep the robot stationary, and remove other
colored targets. Place the cylinder at five to seven horizontal bearings and
at approximately 0.8 m, 1.5 m, and 2.5 m. Cover both image edges and the center.

With the camera, snapshot builder, and detector running, start the capture
node on the robot:

```bash
ros2 launch color_object_sorter camera_lidar_calibration.launch.py \
  output_file:=/home/sunrise/camera_lidar_calibration.csv
```

Inspect the current messages and select exactly one pair. Zero means automatic
selection and is accepted only when exactly one candidate/detection remains:

```bash
ros2 param set /camera_lidar_calibration_capture candidate_id 3
ros2 param set /camera_lidar_calibration_capture detection_track_id 7
ros2 param set /camera_lidar_calibration_capture expected_color red
```

At each physical position, wait for stable data and capture one row:

```bash
ros2 service call /camera_lidar_calibration/capture std_srvs/srv/Trigger '{}'
```

Do not repeatedly capture the same position. The fitter requires at least 12
geometrically diverse rows, at least 0.50 rad angular coverage, and at least
0.50 m distance coverage.

## Fit and review

Copy the CSV to the development workspace and run:

```bash
ros2 run color_object_sorter camera_lidar_calibrator \
  /path/to/camera_lidar_calibration.csv
```

Review training RMSE, maximum residual, leave-one-out RMSE, and the monotonicity
check. Copy the suggested coefficients into `config/perception.yaml` only after
review, then explicitly change `projection_model` to `polynomial_range`.

## Accepted calibration: 2026-09-07

The stationary calibration produced 13 valid correspondences spanning
0.668919 rad and 1.589252 m. Training RMSE was 0.010286 normalized image x,
maximum residual was 0.017266, and leave-one-out RMSE was 0.019493. The mapping
passed the monotonicity check and is enabled with:

```yaml
projection_model: polynomial_range
projection_coefficients: [0.0156991059397, -2.55105603026, -0.0974005459428, -1.14019421997, -0.0244140933769, -0.198560842383]
```

The source CSV remains on the robot at
`/home/sunrise/camera_lidar_calibration_v2.csv` and is not committed because it
is machine-specific calibration evidence.

At runtime, inspect individual assignments without opening a video stream:

```bash
ros2 topic echo /cylinder_validation/projection_diagnostics
```

Each JSON record contains the candidate ID, bearing, range, predicted image x,
assigned visual track/color, and residual. Recalibrate after changing camera
mounting, image resolution, cropping, or scaling.
