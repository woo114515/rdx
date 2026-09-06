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

As of 2026-09-06, the official Task 3 colors are blue, a replacement green,
and red. The former pink and green objects and their calibration samples are
legacy data and must not be mixed into the new CSV. Blue may be left unchanged,
but collecting a small blue validation set under the same lighting is useful.

## View detector output

With the camera and `color_object_detector` running, open the annotated stream
in the graphical VM with:

```bash
ros2 run color_object_sorter color_sorter_debug_viewer
```

The viewer uses BEST_EFFORT sensor-data QoS for the compressed debug topic and
does not depend on the VM's conflicting `rqt_image_view` compressed transport
plugin. Press `q` or Escape to close it.
