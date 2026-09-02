#!/usr/bin/env bash

# Collect read-only RDK X5, device, and ROS information.
# This script deliberately does not start drivers or publish ROS messages.

set -o pipefail

section() {
  printf '\n[%s]\n' "$1"
}

run_if_available() {
  local command_name="$1"
  shift

  if command -v "$command_name" >/dev/null 2>&1; then
    "$command_name" "$@" 2>&1 || true
  else
    printf '%s is not installed\n' "$command_name"
  fi
}

section "Timestamp"
date --iso-8601=seconds 2>&1 || true

section "Board"
if [[ -r /proc/device-tree/model ]]; then
  tr -d '\0' </proc/device-tree/model
  printf '\n'
fi
run_if_available uname -a

section "Operating system"
if [[ -r /etc/os-release ]]; then
  sed -n '1,40p' /etc/os-release
fi
if [[ -r /etc/version ]]; then
  printf 'RDK version file: '
  sed -n '1p' /etc/version
fi
run_if_available rdkos_info

section "CPU and memory"
run_if_available lscpu
run_if_available free -h

section "USB and serial devices"
run_if_available lsusb
for device_path in /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/*; do
  if [[ -e "$device_path" ]]; then
    ls -l "$device_path"
  fi
done

section "Camera devices"
for device_path in /dev/video* /dev/media*; do
  if [[ -e "$device_path" ]]; then
    ls -l "$device_path"
  fi
done
run_if_available v4l2-ctl --list-devices

section "ROS environment"
ros_setup=""
for setup_candidate in \
  /opt/tros/humble/setup.bash \
  /opt/tros/setup.bash \
  /opt/ros/humble/setup.bash \
  /opt/ros/foxy/setup.bash; do
  if [[ -r "$setup_candidate" ]]; then
    ros_setup="$setup_candidate"
    break
  fi
done

if [[ -n "$ros_setup" ]]; then
  printf 'Sourcing %s\n' "$ros_setup"
  # shellcheck disable=SC1090
  source "$ros_setup"
else
  printf 'No known ROS setup file found\n'
fi

printf 'ROS_DISTRO='
printenv ROS_DISTRO 2>/dev/null || printf 'unset\n'
printf 'TROS_DISTRO='
printenv TROS_DISTRO 2>/dev/null || printf 'unset\n'

if command -v ros2 >/dev/null 2>&1; then
  export ROS2CLI_DISABLE_DAEMON=1

  section "Relevant ROS packages"
  ros2 pkg list 2>&1 \
    | grep -Ei 'hobot|oradar|ms200|yahboom|rosmaster|slam|nav2' \
    || true

  section "Active ROS nodes"
  ros2 node list 2>&1 || true

  section "Active ROS topics"
  ros2 topic list -t 2>&1 || true
else
  printf 'ros2 is not installed or not sourced\n'
fi
