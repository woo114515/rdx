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

The following pink result is retained only as historical calibration evidence;
pink is no longer a Task 3 target after the 2026-09-06 color replacement. In a
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

The node rejects scans older than 0.25 s. It first extracts all contiguous
clusters containing at least two returns whose adjacent ranges differ by no
more than 0.15 m. It then globally assigns visual detections to clusters while
enforcing one detection per cluster and one cluster per detection. The search
window is at least ±4° and expands with visual bounding-box width so a merged
wide contour can expose multiple candidates. An unmatched or ambiguous object
remains in the output with `matched=false` and `distance=NaN`.
Published bearings are normalized to `[-pi, pi]` even though the MS200P scan
uses approximately `[0, 2*pi]`.
For a successful synchronized fusion, the output header carries the LiDAR scan
timestamp and `lidar_link` frame because bearing and distance are LiDAR
measurements. If no fresh scan exists, unmatched output retains the camera
header and must not be interpreted as a LiDAR-frame position.

The former green-recognition issue applies to the abandoned green cylinders.
Replacement-green and red have new sampled HSV ranges but still require live
acceptance. Fusion does not correct color misclassification.

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

The implementation is intended for objects that are visibly separated and
have clearly different bearings. Stage-1 one-to-one allocation prevents a
single LiDAR cluster from being reported as two valid objects, but overlap is
still not fully recoverable:

- Different-color objects may remain separately detectable only when each has
  enough visible image area.
- Overlapping objects of the same color usually merge into one HSV contour and
  are therefore reported as one visual object.
- The 2D LiDAR normally observes only the nearer surface when one cylinder
  occludes another at the same bearing; it cannot provide an independent range
  for the hidden cylinder.
- Two visual detections competing for one similarly plausible cluster are
  marked `ambiguous`; neither is permitted to use that range for confirmation.
- One wide visual contour spanning multiple similarly plausible clusters is
  also marked `ambiguous` instead of silently selecting the nearest surface.
- The temporal confirmer associates by bearing and distance so identities may
  merge or swap during crossing and short occlusion. Conflicting colors often
  reduce `color_consistency` and conservatively prevent confirmation, but this
  is rejection rather than correct multi-object separation.

Consequently, overlapping, merged, crossing, or occluded observations must not
trigger approach or sorting motion. `association_status` reports `matched`,
`unmatched`, `ambiguous`, or `stale_scan`; temporal diagnostic state reports
`tentative`, `confirmed`, `ambiguous`, or `occluded`. Hidden tracks are retained
only inside the short confirmation window and are removed from the confirmed
output immediately. Multi-view observation is still required to separate a
merged contour or reveal a fully hidden cylinder.

The reported physical spacing between adjacent cylinders was corrected on
2026-09-06 from approximately 15 cm to approximately 30 cm. It is not yet known
whether this is center-to-center distance or clear edge-to-edge distance. The
20 cm-wide robot must not treat the nominal 30 cm as a traversable gap until
that definition, cylinder diameter, localization error, and swept footprint
have been measured. The larger spacing helps perception but does not remove
the one-to-one association and occlusion requirements above.

## Dense-field association correction (2026-09-06)

A stationary six-cylinder test produced all six correct vision detections but
only five map landmarks. A 15-second live probe evaluated 100 frames and
observed 94 ambiguous events. It captured two concrete causes:

- valid cylinder returns around 1.5 m competed with wall/background fragments
  around 3.4--11.9 m because the old candidate logic used bearing alone;
- two nearby visual detections sometimes competed for one range-contiguous
  cluster because adjacent samples with similar ranges were always joined.

Fusion now keeps only candidates within `maximum_depth_gap` of the nearest
surface in each visual window. It also divides a continuous return wider than
`maximum_cluster_angular_span` before global allocation. Defaults are 0.35 m
and 4 degrees. These filters run before the existing one-to-one and ambiguity
checks; they do not force uncertain matches or weaken rejection thresholds.
