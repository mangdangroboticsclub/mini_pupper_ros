#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
