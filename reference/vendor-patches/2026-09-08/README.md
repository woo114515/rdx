# GMapping timing fixes

`slam-gmapping-map-update-throttle.patch` applies to the root of the vendor
workspace `src` directory. The vendor ROS 2 port declared
`last_map_update` inside `laserCallback()`, so every callback reset it to zero
and defeated the configured `map_update_interval`.

The patch moves the timestamp into `SlamGmapping` persistent state and declares
the two timing parameters that this vendor port previously never read. The first
map is still generated immediately; later full-map rebuilds obey the configured
interval. It also preserves the fractional `transform_publish_period` when
future-dating `map -> odom`; the old integer conversion silently changed 0.05
seconds to zero.

Only `map_update_interval` and `transform_publish_period` are connected to ROS
parameters. The remaining vendor defaults deliberately stay unchanged so this
timing repair does not also retune scan matching or the particle filter.

Apply only after stopping the mapping launch and backing up both affected
files:

```bash
cd /home/sunrise/yahboomcar_ws/src
patch -p1 < /path/to/slam-gmapping-map-update-throttle.patch
cd /home/sunrise/yahboomcar_ws
colcon build --symlink-install --packages-select slam_gmapping
```

This patch affects the live GMapping build path. Task 2 navigation from a saved
map uses `map_server` and AMCL, so it does not execute this code. After restart,
verify `/scan` remains close to 10 Hz and `/map` advances at approximately the
configured five-second interval while GMapping accepts scans.
