#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "/ros_workspace/install/setup.bash" --

if [ "$(id -u)" -eq 0 ]; then
    ip route del default
    ip route add default via "192.168.${SUBNET:-33}.1"
fi

exec "$@"
