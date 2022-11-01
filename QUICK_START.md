# Quick Start with Mini Pupper Simulation

## Prerequisite 
- Ubuntu 20.04

## 1. Install ROS Noetic 

First, install ROS Noetic Desktop-Full by following the page below.
https://wiki.ros.org/noetic/Installation/Ubuntu


## 2. Install Common ROS Tools

```sh
sudo apt-get update
sudo apt-get install -y python3-vcstool python3-rosdep ninja-build stow
```

## 3. Install Mini-Pupper Packages and Dependencies


```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/mangdangroboticsclub/mini_pupper_ros.git
vcs import < minipupper_ros/.minipupper.repos --recursive
```

```sh
cd ..
rosdep install --from-paths . --ignore-src -r -y

```

### 4. Install cartographer

```sh
mkdir -p ~/carto_ws/src
cd ~/carto_ws
vcs import src --input https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
# Hot fix for Ubuntu 20.04. See https://github.com/cartographer-project/cartographer_ros/pull/1745
sed -i -e "s%<depend>libabsl-dev</depend>%<\!--<depend>libabsl-dev</depend>-->%g" src/cartographer/package.xml
sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
src/cartographer/scripts/install_abseil.sh
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
```

## 4. Build

```sh
catkin_make
source $YOUR_WS/devel/setup.bash
```

## 5. Run Simulation in Gazebo 

Terminal 1
```sh
roslaunch mini_pupper_bringup bringup.launch
```

Terminal 2
```sh
roslaunch mini_pupper_navigation navigate.launch
```

Terminal 3
```sh
roslaunch champ_teleop teleop.launch
```



