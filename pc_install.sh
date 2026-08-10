#!/bin/bash
# Mini Pupper PC Installation Script
# Upgraded from ROS2 Humble to ROS2 Jazzy

set -e

echo "Installing Mini Pupper dependencies for ROS2 Jazzy..."

# Install ROS2 Jazzy if not already installed
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 not found. Please install ROS2 Jazzy first."
    echo "See: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-cartographer-ros \
    ros-jazzy-launch-testing-ament-cmake \
    ros-jazzy-launch-testing-ros \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "Installation complete. Please source your workspace."
echo "source /opt/ros/jazzy/setup.bash"
