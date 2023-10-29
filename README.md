[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2/LICENSE)
&nbsp;
[![Twitter URL](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Ftwitter.com%2FLeggedRobot)](https://twitter.com/LeggedRobot)

# Mini Pupper ROS 2 Humble

This branch is only to show how to use a script to make Mini Pupper 2 dance, just like the workshop on AWS re:Invent 2022.
Supported versions

* Ubuntu 22.04 + ROS 2 Humble

## 1. Installation

### 1.1 PC Setup

PC Setup corresponds to PC (your desktop or laptop PC) for controlling Mini Pupper remotely or execute simulator.  
__Do not apply these PC Setup commands to your Raspberry Pi on Mini Pupper.__

Ubuntu 22.04 + ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the below [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

```sh
cd ~
sudo apt update
git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
~/ros2_setup_scripts_ubuntu/ros2-humble-ros-base-main.sh
source /opt/ros/humble/setup.bash
```

After ROS 2 installation, download the Mini Pupper ROS package in the workspace.

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros

# Move mini_pupper_dance and mini_pupper_interfaces folders into ~/ros2_ws/src

vcs import < mini_pupper_ros/.minipupper.repos --recursive
```

Build and install all ROS packages.

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-teleop-twist-keyboard
colcon build --symlink-install
```

### 1.2 Mini Pupper Setup

Mini Pupper Setup corresponds to the Raspberry Pi on your Mini Pupper.  
Ubuntu 22.04 is required.

Before installation, you need to install the BSP(board support package) repo for your [Mini Pupper 2](https://github.com/mangdangroboticsclub/mini_pupper_2_bsp) or [Mini Pupper](https://github.com/mangdangroboticsclub/mini_pupper_bsp.git).

After installing the driver software, install ROS 2. ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

After that, please follow the below steps to install this repo.

```sh
cd ~
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
cd mini_pupper_ros
./install.sh
```

## 2. Quick Start

## 2.1 Mini Pupper

```sh
# Terminal 1 (ssh)
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

```sh
# Terminal 2 (ssh)
. ~/ros2_ws/install/setup.bash 
ros2 launch mini_pupper_music music.launch.py
```

## 2.2 PC (Or Mini Pupper)
```sh
# Terminal 3 (ssh)
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_dance dance.launch.py
```

## 2.3 How to modify

### mini_pupper_dance
Includes two Python scripts in mini_pupper_dance/mini_pupper/dance folder. </br>
dance_client.py is the client reading and sending dance commands. </br>
dance_server.py is the server receiving dance commands and executing them. You can add more dance functions in dance_server.py </br>
pose_controller.py is a pose controller for mini pupper. You don't need to modify this. </br>
episode.py is the dancing episode. You should edit your dancing episode here.

### mini pupper_interfaces
This package includes the customized dance command service message.
