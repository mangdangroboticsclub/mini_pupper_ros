# Quick Start with Mini Pupper Simulation

You can play with Mini Pupper with only your laptop or PC. Let's learn about Robotics by running the simulation in Gazebo.   

![nav](../imgs/instruction.gif)

## Pre-requisite 

- Ubuntu 20.04

## 1. Install ROS Noetic 

First, install ROS Noetic Desktop-Full by following the page below.
https://wiki.ros.org/noetic/Installation/Ubuntu


## 2. Install Cartographer

```sh
sudo apt-get update
sudo apt-get install -y python3-vcstool python3-rosdep ninja-build stow git
mkdir -p ~/carto_ws/src
cd ~/carto_ws
vcs import src --input https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
```

```sh
sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
src/cartographer/scripts/install_abseil.sh
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
catkin_make_isolated --install --use-ninja
```

## 3. Install Mini-Pupper and Other Dependencies

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/mangdangroboticsclub/mini_pupper_ros.git
vcs import < mini_pupper_ros/.minipupper.repos --recursive
```

```sh
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
source ~/carto_ws/install_isolated/setup.bash
catkin_make
```

## 4. Run Simulation in Gazebo 


```sh
# Terminal 1
source ~/catkin_ws/devel/setup.bash
roslaunch mini_pupper_gazebo gazebo.launch
```

```sh
# Terminal 2
source ~/catkin_ws/devel/setup.bash
roslaunch mini_pupper_navigation navigate.launch use_odom:=true
```

```sh
# Terminal 3
source ~/catkin_ws/devel/setup.bash
roslaunch champ_teleop teleop.launch
```



