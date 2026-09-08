# GMapping map-update throttle fix

`slam-gmapping-map-update-throttle.patch` applies to the root of the vendor
workspace `src` directory. The vendor ROS 2 port declared
`last_map_update` inside `laserCallback()`, so every callback reset it to zero
and defeated the configured `map_update_interval`.

The patch moves the timestamp into `SlamGmapping` persistent state. The first
map is still generated immediately; later full-map rebuilds obey the configured
interval. Scan matching, TF calculation, map contents, topics, services, and
navigation controller interfaces are unchanged.

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
