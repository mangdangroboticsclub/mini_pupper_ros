#!/bin/bash
######################################################################################
# ROS2
#
# This stack will consist of ROS2 install
#
# To install
#    ./pc_install.sh
######################################################################################

# Update package lists
cd ~
sudo apt update

ROS_DISTRO=${ROS_DISTRO:-jazzy}
ROS_SETUP_SCRIPT=${ROS_SETUP_SCRIPT:-ros2-${ROS_DISTRO}-ros-base-main.sh}

# Install ROS 2 setup scripts
if ! [ -d "ros2_setup_scripts_ubuntu" ]; then
  git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
fi
~/ros2_setup_scripts_ubuntu/${ROS_SETUP_SCRIPT}
source /opt/ros/${ROS_DISTRO}/setup.bash

# Create ROS 2 workspace and clone Mini Pupper ROS repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if ! [ -d "mini_pupper_ros" ]; then
  git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
fi
vcs import < mini_pupper_ros/.minipupper.repos --recursive

# Install dependencies and build the ROS 2 packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-${ROS_DISTRO}-teleop-twist-keyboard ros-${ROS_DISTRO}-teleop-twist-joy
sudo apt install -y ros-${ROS_DISTRO}-v4l2-camera ros-${ROS_DISTRO}-image-transport-plugins
sudo apt install -y ros-${ROS_DISTRO}-rqt*
pip3 install simple_pid
colcon build --symlink-install
