# Task 3 camera–LiDAR fusion

## Purpose

The camera determines object color; the MS200P LiDAR supplies bearing and
distance. This stage is perception-only and publishes no motion commands.

## Stationary calibration

On 2026-09-06, a blue cylinder near image center was measured at about 0° and
1.22 m. A pink cylinder at image `normalized_x=-0.736` was measured at about
+15° and 1.25 m. The provisional conversion is:

```text
bearing_rad = -0.3926990817 * (normalized_x - (-0.069))
```

Positive LiDAR angles point left. This is a two-point approximation and must be
recalibrated if camera mounting, resolution, or cropping changes. The observed
effective horizontal field of view is only about 45°, so searching ultimately
requires robot rotation.

Pink detection is less reliable than blue under the tested lighting. In a
five-second stationary sample, pink appeared in 31 of 40 detection frames and
had a median confidence of about 0.68, while blue appeared in all 40 frames
with median confidence about 0.93. A pink detection must therefore be treated
as a candidate rather than allowing one frame to trigger motion. The sorting
stage should require several temporally consistent observations and a valid
LiDAR association; the exact acceptance threshold still requires more real
robot testing.

## ROS graph

- Inputs: `/color_sorter/detections`, `/scan`
- Output: `/color_sorter/localized_objects`
- Confirmed output: `/color_sorter/confirmed_objects`
- Diagnostic output: `/color_sorter/confirmation_status`
- Executable: `color_lidar_fusion`
- Launch: `ros2 launch color_object_sorter perception.launch.py`

The node rejects scans older than 0.25 s. Within ±4° of the projected camera
bearing it selects the nearest contiguous cluster containing at least two
returns whose adjacent ranges differ by no more than 0.15 m. An unmatched
object remains in the output with `matched=false` and `distance=NaN`.
Published bearings are normalized to `[-pi, pi]` even though the MS200P scan
uses approximately `[0, 2*pi]`.
For a successful synchronized fusion, the output header carries the LiDAR scan
timestamp and `lidar_link` frame because bearing and distance are LiDAR
measurements. If no fresh scan exists, unmatched output retains the camera
header and must not be interpreted as a LiDAR-frame position.

Green recognition remains deferred: the green cylinder is not reliably
separated from the floor/background. Fusion does not correct color
misclassification.

## Multi-frame confirmation

`temporal_object_confirmation` keeps the raw localized topic available for
diagnosis and publishes only confirmed tracks on the confirmed topic. It uses
a one-second rolling window and requires at least six observations, detection
in 60% of input frames, 80% agreement on color, LiDAR matches for 50% of the
winning-color observations, bearing standard deviation no greater than 3°, and
distance standard deviation no greater than 0.10 m. These are initial safety
thresholds, not final calibrated values.

Tracks are associated by bearing and distance rather than trusting the camera
tracker ID permanently. This permits a short visual dropout to receive a new
camera ID without immediately discarding its history. A single frame can never
appear on `/color_sorter/confirmed_objects`, and this package still has no
motion publisher.

The diagnostic topic contains both passing and failing candidates with their
metrics and `confirmed` flag. It is for tuning and observation only. Any future
motion-capable consumer must subscribe to `/color_sorter/confirmed_objects`,
not the diagnostic or raw localized topics.

## Overlap and occlusion limitations

The current implementation is intended for objects that are visibly separated
and have clearly different bearings. Overlapping objects are not yet handled
reliably:

- Different-color objects may remain separately detectable only when each has
  enough visible image area.
- Overlapping objects of the same color usually merge into one HSV contour and
  are therefore reported as one visual object.
- The 2D LiDAR normally observes only the nearer surface when one cylinder
  occludes another at the same bearing; it cannot provide an independent range
  for the hidden cylinder.
- Fusion currently associates every visual detection independently. Two nearby
  visual detections can therefore be assigned the same LiDAR cluster in one
  frame. `matched=true` alone does not prove that the LiDAR distinguished both
  objects.
- The temporal confirmer associates by bearing and distance so identities may
  merge or swap during crossing and short occlusion. Conflicting colors often
  reduce `color_consistency` and conservatively prevent confirmation, but this
  is rejection rather than correct multi-object separation.

Consequently, overlapping, merged, crossing, or occluded observations must not
trigger approach or sorting motion. A later revision should extract LiDAR
clusters first and perform one-to-one global assignment, ensuring that one
cluster is used by at most one visual object per frame. It should also expose
explicit `occluded` and `ambiguous` states and retain hidden tracks only for a
short prediction interval. Until that work is completed, Task 3 placement and
search should keep candidate cylinders spatially separated.
