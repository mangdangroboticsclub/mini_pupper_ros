#!/bin/bash
set -e

# Source ROS2
ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source workspace
source /ros2_ws/install/setup.bash

# Default to launching tracking demo if no args provided
if [ $# -eq 0 ]; then
    exec ros2 launch mini_pupper_tracking webcam_tracking.launch.py
else
    exec "$@"
fi
