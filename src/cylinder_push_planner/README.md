# cylinder_push_planner

This package converts a locked, validated cylinder snapshot into motion-free
planning geometry. It does not publish `Twist` or invoke Nav2.

Start the complete motion-free Task 3 pipeline with one launch command:

```bash
ros2 launch cylinder_push_planner task3_planning.launch.py
ros2 service call /cylinder_snapshot/start std_srvs/srv/Trigger '{}'
ros2 service call /cylinder_push_plan/generate std_srvs/srv/Trigger '{}'
```

The launch starts the selection planner, LiDAR snapshot builder, color detector,
and candidate validator. Do not also start `snapshot.launch.py` or
`lidar_first_validation.launch.py`, because that would create duplicate nodes.
The camera, LiDAR, TF, and mapping/base launch remain external hardware
prerequisites.

The locked `/cylinder_snapshot/validated_objects` result uses reliable,
transient-local QoS. A planner or diagnostic subscriber that starts after the
one-shot automatic lock therefore still receives the latest result.

RViz can display `/cylinder_push_plan/markers`. The orange circle is the
initial cylinder envelope, the red sphere is the selected right-side target,
and the blue arrow is an outside-envelope staging pose facing the target.

This first increment deliberately stops before trajectory generation. A staging
pose is not yet a collision-checked Nav2 goal, and no robot motion should be
started from it. Color-to-destination directions will be configured rather than
hard-coded when the field convention is confirmed.
