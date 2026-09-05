# Competition arena map

`competition_arena_2026-09-05.yaml` and its PGM image are the map recorded in
the real competition arena on 2026-09-05.

- Size: 384 x 672 pixels
- Resolution: 0.05 m/pixel
- Origin: `[-10, -24.4, 0]`
- ROS frame: `map`

After building and sourcing the workspace, obtain the installed map path with:

```bash
ros2 pkg prefix rdx_navigation
```

Pass the resulting package-share map YAML to the navigation launch. Verify the
robot footprint, AMCL initial pose, laser alignment, and waypoint coordinates in
the arena before enabling motion.
