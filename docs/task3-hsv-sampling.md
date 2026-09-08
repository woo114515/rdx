# Task 3 HSV Sampling

The HSV sampler subscribes to the same compressed image as the detector and
stores labelled pixel patches in CSV form. It never publishes motion commands.

Run it in a graphical ROS 2 environment:

```bash
ros2 run color_object_sorter hsv_sampler --ros-args \
  -p output_file:=$HOME/hsv_samples.csv \
  -p patch_radius:=5
```

Select the active label before clicking inside a uniformly coloured area:

- `b`: blue target
- `g`: green target
- `k`: black target
- `r`: red target
- `y`: yellow distractor (not a target)
- `n`: background or another distractor
- `u`: undo the most recent patch
- `s`: save without quitting
- `q` or Escape: save and quit

With the default radius, each click records an 11 by 11 patch. Sample target
highlights, normal regions, and shadows at near, medium, and far distances.
Avoid object boundaries. Collect background and yellow samples so target ranges
can be separated from common false positives.

The output uses OpenCV HSV units: hue 0-179 and saturation/value 0-255. Runtime
CSV data belongs in an operator data directory and must not be committed by
default.

## Analyse the CSV

The repository includes a repeatable analyser. It uses the 2nd and 98th
percentiles instead of raw minima and maxima, adds small configurable margins,
handles hue as a circular value (including red across 179/0), and reports how
many labelled background pixels the proposed range would accept.

After building and sourcing the workspace, run:

```bash
ros2 run color_object_sorter hsv_analyzer "$HOME/hsv_samples.csv"
```

To analyse only the official colors:

```bash
ros2 run color_object_sorter hsv_analyzer "$HOME/hsv_samples.csv" \
  --labels blue green red
```

The output contains click counts, pixel counts, background overlap, and a
suggested ROS parameter block. Review that report before copying values into
`config/perception.yaml`; the analyser deliberately does not modify the active
configuration automatically.

As of the latest 2026-09-07 update, the official Task 3 colors are blue, a new
green, and red. Pink and the earlier green and black targets are retired. Their
samples must not be relabelled as background: keep them under their original
labels so they are excluded from both the target fit and background-overlap
calculation. Blue and red retain their previously accepted ranges.

Black is classified primarily by low value (`V`). Hue and saturation are
unstable for nearly black pixels, so the analyser deliberately proposes the
full Hue/Saturation range and derives only the upper Value limit. Collect dark
backgrounds and shadows with `n`; background overlap is an essential rejection
check for black and must be reviewed before deployment.

The retained black/background dataset contains 65 black clicks (7865 pixels)
and 64 background clicks (7744 pixels). It remains useful historical evidence
but black is not present in the active detector or configured inventory.

The new-green dataset contains 61 green clicks (7381 pixels) and 57 background
clicks (6875 pixels). The analyser initially proposed `[76, 98, 67]` through
`[82, 207, 117]`. For live acceptance the range was deliberately widened to
`[74, 70, 55]` through `[85, 230, 135]`; it covers 99.69% of sampled green
pixels and none of the sampled background pixels. A wider candidate was not
deployed because its small recall gain did not justify the additional
generalisation risk.

On 2026-09-07 an additional green-only dataset (`green_resample.csv`) was
collected to cover the newly observed appearance without replacing the
accepted range. It contains 98 clicks and 11,858 pixels. The repeatable
analyser proposed `[63, 73, 52]` through `[86, 255, 133]` with no background
overlap measurable because this CSV contains no background samples. This is
therefore configured as a second green range alongside the previous range;
its false-positive rate must be checked in the live acceptance image.

## View detector output

With the camera and `color_object_detector` running, open the annotated stream
in the graphical VM with:

```bash
ros2 run color_object_sorter color_sorter_debug_viewer
```

The viewer uses BEST_EFFORT sensor-data QoS for the compressed debug topic and
does not depend on the VM's conflicting `rqt_image_view` compressed transport
plugin. Press `q` or Escape to close it.
