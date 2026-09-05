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
- `p`: pink target
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
