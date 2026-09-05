#!/usr/bin/env bash

set -u

missing=0

check_name() {
    kind="$1"
    name="$2"
    values="$3"
    if printf '%s\n' "$values" | grep -Fxq "$name"; then
        printf 'OK   %-7s %s\n' "$kind" "$name"
    else
        printf 'MISS %-7s %s\n' "$kind" "$name"
        missing=1
    fi
}

topics="$(ros2 topic list 2>/dev/null)"
actions="$(ros2 action list 2>/dev/null)"

check_name topic /map "$topics"
check_name topic /amcl_pose "$topics"
check_name topic /odom_raw "$topics"
check_name topic /scan "$topics"
check_name action /navigate_to_pose "$actions"

printf '\nNav2 must publish velocity only to:\n'
printf '  /rdx_sorting/nav_cmd_vel_request\n'
printf 'Only rdx_sorting_safety may publish directly to /cmd_vel.\n\n'
ros2 topic info /cmd_vel --verbose 2>/dev/null || true

exit "$missing"
