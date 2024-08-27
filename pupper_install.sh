#!/bin/bash
######################################################################################
# ROS2
#
# This stack will consist of ROS2 install
#
# To install
#    ./pupper_install.sh
######################################################################################

set -e
echo "setup.sh started at $(date)"

###### work in progress #######

# check Ubuntu version
source /etc/os-release

if [[ $UBUNTU_CODENAME != 'jammy' ]]
then
    echo "Ubuntu 22.04 LTS (Jammy Jellyfish) is required"
    echo "You are using $VERSION"
    exit 1
fi

############################################
# wait until unattended-upgrade has finished
############################################
tmp=$(ps aux | grep unattended-upgrade | grep -v unattended-upgrade-shutdown | grep python | wc -l)
[ $tmp == "0" ] || echo "waiting for unattended-upgrade to finish"
while [ $tmp != "0" ];do
sleep 10;
echo -n "."
tmp=$(ps aux | grep unattended-upgrade | grep -v unattended-upgrade-shutdown | grep python | wc -l)
done

cd ~
sudo apt-get update
sudo apt -y install python3-pip python3-venv python3-virtualenv

#Auto install ROS2 Humble
if ! [ -d "ros2_setup_scripts_ubuntu" ]; then
  git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
fi
~/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash

#clone mini pupper 2 ros2 repo
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if ! [ -d "mini_pupper_ros" ]; then
  git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
fi
vcs import < mini_pupper_ros/.minipupper.repos --recursive
# compiling gazebo and cartographer on Raspberry Pi is not recommended
touch champ/champ/champ_gazebo/AMENT_IGNORE
touch champ/champ/champ_navigation/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE

# install dependencies without unused heavy packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=rviz2 --skip-keys=gazebo_plugins --skip-keys=velodyne_gazebo_plugins
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-teleop-twist-joy
sudo apt install -y ros-humble-v4l2-camera ros-humble-image-transport-plugins
pip3 install simple_pid

#colcon build --symlink-install
MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install

# show IP address on LCD when boot up
cd ~/mini_pupper_ros/
sudo mv robot.service /etc/systemd/system/
sudo mkdir -p /var/lib/minipupper/
sudo mv run.sh /var/lib/minipupper/
sudo mv show_ip.py /var/lib/minipupper/
sudo systemctl daemon-reload
sudo systemctl enable robot

echo "setup.sh finished at $(date)"
sudo reboot
