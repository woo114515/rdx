#!/usr/bin/env bash
# Source this file in every Task 3 shell on the robot.

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "Source this file instead of executing it:" >&2
    echo "  source ${BASH_SOURCE[0]}" >&2
    exit 2
fi

if [[ -f /opt/tros/humble/setup.bash ]]; then
    source /opt/tros/humble/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "ROS 2/TROS Humble setup.bash was not found" >&2
    return 1
fi

# The MS200P driver is built in Yahboom's separate hardware workspace.
if [[ -f /home/sunrise/software/library_ws/install/setup.bash ]]; then
    source /home/sunrise/software/library_ws/install/setup.bash
fi

if [[ -f /home/sunrise/yahboomcar_ws/install/setup.bash ]]; then
    source /home/sunrise/yahboomcar_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
unset ROS_DISCOVERY_SERVER
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

echo "Task 3 ROS environment: domain=${ROS_DOMAIN_ID}, transport=UDPv4"
