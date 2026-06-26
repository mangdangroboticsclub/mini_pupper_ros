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

# Install ROS 2 Jazzy setup scripts ------------ RECHECK REQUIRED - Error to read the link 

# if ! [ -d "ros2_setup_scripts_ubuntu" ]; then
#   git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
# fi
# ~/ros2_setup_scripts_ubuntu/ros2-jazzy-ros-base-main.sh
source /opt/ros/jazzy/setup.bash

# Create ROS 2 workspace and clone Mini Pupper ROS repository
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if ! [ -d "mini_pupper_ros" ]; then
  git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-jazzy mini_pupper_ros
else
  echo "mini_pupper_ros folder already exists. Pulling latest changes..."
  cd mini_pupper_ros
  git pull origin ros2-jazzy
  cd ..
fi
vcs import < mini_pupper_ros/.minipupper.repos --recursive

# Install dependencies and build the ROS 2 packages
cd ~/ros2_ws
sudo apt install python3-rosdep
sudo rosdep fix-permissions
rosdep update
rosdep install --from-paths src --ignore-src -r -y
sudo apt install -y ros-jazzy-teleop-twist-keyboard ros-jazzy-teleop-twist-joy
sudo apt install -y ros-jazzy-v4l2-camera ros-jazzy-image-transport-plugins
sudo apt install -y ros-jazzy-rqt*
sudo apt install python3-pip -y
pip3 install --user --break-system-packages simple_pid

colcon build --symlink-install

# Add GZ_SIM_RESOURCE_PATH to ~/.bashrc if not already present
LINE='export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros2_ws/src/mini_pupper_ros'
# Check if the line already exists
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc