# Fast DDS transport for Task 3

## Symptom and diagnosis

During the 2026-09-08 field test, several ROS 2 commands printed:

```text
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port32173:
open_and_lock_file failed -> Function open_port_internal
```

The affected service eventually became available when the process fell back
to UDP, but discovery was slow and repeated commands appeared to hang.  After
all ROS processes stopped, `/dev/shm` contained no Fast DDS objects and a fresh
default-transport graph query succeeded.  This identifies the incident as a
transient Fast DDS shared-memory port-lock conflict, not a failure of the
robot's IP connection or UDP transport.

Fast DDS normally creates both UDPv4 and SHM builtin transports, preferring SHM
for same-host peers.  Its supported `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` setting
creates only UDPv4 and therefore avoids the failing SHM initialization path.

## Project-scoped fix

`task3_planning.launch.py` and `task3_visualization.launch.py` set
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4` before starting any child node.  The setting
is scoped to those launch processes and does not globally change Task 2 or
other robot workloads.  It can be overridden explicitly for a controlled
same-host test:

```bash
ros2 launch cylinder_push_planner task3_planning.launch.py \
  fastdds_builtin_transports:=DEFAULT
```

Direct `ros2 service`, `topic`, and `param` commands create their own DDS
participants, so interactive terminals should source the supplied helper:

```bash
source /home/sunrise/yahboomcar_ws/task3_ros_env.sh
```

The deployed helper is copied from the repository's `scripts/` directory.  It
sources TROS/ROS Humble, sources `~/yahboomcar_ws/install`
when present, selects domain 99 unless already set, removes a stale discovery
server setting, and selects UDPv4.

Do not delete `/dev/shm` files while ROS processes are running.  If the same
error occurs outside the Task 3 launch, stop the affected ROS processes first,
verify that no participant still owns the files, and only then investigate
stale shared-memory objects and file ownership.
