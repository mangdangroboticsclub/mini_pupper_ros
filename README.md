[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2/LICENSE)
&nbsp;
[![Twitter URL](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Ftwitter.com%2FLeggedRobot)](https://twitter.com/LeggedRobot)

# Mini Pupper ROS 2

Supported versions

* Ubuntu 22.04 + ROS 2 Humble

## 1. Installation

* To use Gazebo simulator, "1.1 PC Setup" is required.
* To control Mini Pupper, "1.2 Mini Pupper Setup" is required.
* To control Mini Pupper using visualize tools, "1.1 PC Setup" and "1.2 Mini Pupper Setup" is required.

### 1.1 PC Setup

PC Setup corresponds to PC (your desktop or laptop PC) for controlling Mini Pupper remotely or execute simulator.  
__Do not apply these PC Setup commands to your Raspberry Pi on Mini Pupper.__

Ubuntu 22.04 + ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

After ROS 2 installation, download the Mini Pupper ROS package in the workspace.

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
vcs import < mini_pupper_ros/.minipupper.repos --recursive
```

Build and install all ROS packages.

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-humble-teleop-twist-keyboard
colcon build --symlink-install
```

### 1.2 Mini Pupper Setup

Mini Pupper Setup corresponds to the Raspberry Pi on your Mini Pupper.  
Ubuntu 22.04 is required.

You should first install dependencies of servos, battery moniter and display screen.  
See [mini_pupper_bsp](https://github.com/mangdangroboticsclub/mini_pupper_bsp).

After installing the driver software, install ROS 2. ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

After ROS 2 installation, download the Mini Pupper ROS package in the workspace.

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
vcs import < mini_pupper_ros/.minipupper.repos --recursive
# compiling gazebo and cartographer on Raspberry Pi is not recommended
touch champ/champ/champ_gazebo/AMENT_IGNORE
touch champ/champ/champ_navigation/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE
```

Build and install all ROS packages.

If the Raspberry Pi has less than 4GB memory, try `MAKEFLAGS=-j1 colcon build --executor sequential --symlink-install` instead of `colcon build --symlink-install`

```sh
# install dependencies without unused heavy packages
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=rviz2 --skip-keys=gazebo_plugins --skip-keys=velodyne_gazebo_plugins
sudo apt-get install ros-humble-teleop-twist-keyboard
colcon build --symlink-install
```

## 2. Quick Start

## 2.1 PC
### 2.1.1 Test in RViz

Note: This step is only for PC

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py joint_hardware_connected:=false rviz:=true

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with the keyboard
```

### 2.1.2 Test in Gazebo

Note: This step is only for PC

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_gazebo gazebo.launch.py rviz:=true

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with the keyboard
```

### 2.1.3 Test SLAM (Mapping) in Gazebo

Note: This step is only for PC

- Bring up Gazebo
```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_gazebo gazebo.launch.py
```

- Mapping on PC
```sh
# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation slam.launch.py use_sim_time:=true
```

- Keyboard control  
Use the keyboard to remotely control the Mini Pupper to complete the mapping.
```sh
# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Save the map  
The map will be saved at $HOME.
```sh
# Terminal 4 (on PC)
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${HOME}/cartographer_map.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ${HOME}/cartographer_map
```

#### 2.1.4 Test Navigation in Gazebo

- Replace the map files  
Remember to replace the cartographer_map.pbstream in the maps folder with your new cartographer_map.pbstream first.
```sh
# Terminal 2 (on PC)
cp -f ~/cartographer_map.pgm ~/ros2_ws/src/mini_pupper_ros/mini_pupper_navigation/maps/cartographer_map.pgm
cp -f ~/cartographer_map.pbstream ~/ros2_ws/src/mini_pupper_ros/mini_pupper_navigation/maps/cartographer_map.pbstream
cp -f ~/cartographer_map.yaml ~/ros2_ws/src/mini_pupper_ros/mini_pupper_navigation/maps/cartographer_map.yaml
```

- Bring up Gazebo
```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_gazebo gazebo.launch.py
```

- Localization & Navigation
```sh
# Terminal 3
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation bringup.launch.py use_sim_time:=true
```

- Localization only  
(This step is not necessary if you run `bringup.launch.py`.)
```sh
# Terminal 4
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation localization.launch.py use_sim_time:=true
```

- Navigation only  
(This step is not necessary if you run `bringup.launch.py`.)
```sh
# Terminal 4
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation navigation.launch.py use_sim_time:=true
```

## 2.2 Mini Pupper

### 2.2.1 Test walk

Note: This step is only for Mini Pupper

Open 2 terminals and ssh login to Mini Pupper on both.

```sh
# Terminal 1 (ssh)
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py

# Terminal 2 (ssh)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control Mini Pupper with the keyboard
```
### 2.2.2 Test SLAM (Mapping)

Note: This step requires both PC and Mini Pupper

- Bring up real mini pupper
```sh
# Terminal 1 (ssh to real mini pupper)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

- Mapping on PC
```sh
# Terminal 2 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation slam.launch.py
```

- Keyboard control  
Use the keyboard to remotely control the Mini Pupper to complete the mapping.
```sh
# Terminal 3 (on PC)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Save the map  
The map will be saved at $HOME.
```sh
# Terminal 4 (on PC)
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${HOME}/cartographer_map.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ${HOME}/cartographer_map
```

#### 2.2.3 Test Navigation

- Bring up real mini pupper
```sh
# Terminal 1 (ssh to real mini pupper)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py

```

- Replace the map files  
Remember to replace the cartographer_map.pbstream in the maps folder with your new cartographer_map.pbstream first.
```sh
# Terminal 2 (on PC)
cp -f ~/cartographer_map.pgm ~/ros2_ws/src/mini_pupper_ros/mini_pupper_navigation/maps/cartographer_map.pgm
cp -f ~/cartographer_map.pbstream ~/ros2_ws/src/mini_pupper_ros/mini_pupper_navigation/maps/cartographer_map.pbstream
cp -f ~/cartographer_map.yaml ~/ros2_ws/src/mini_pupper_ros/mini_pupper_navigation/maps/cartographer_map.yaml
```

- Localization
```sh
# Terminal 3 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation localization.launch.py
```

- Navigation
```sh
# Terminal 4 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation navigation.launch.py
```

## License

```
Copyright 2022-2023 MangDang

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## FAQ

* Q. Is Ubuntu 20.04 supported?
  * A. No. Ubuntu 22.04 only for now.
* Q. Is ROS 2 Foxy/Galactic supported?
  * A. No. ROS 2 Humble only for now.
* Q. `colcon build` shows `1 package had stderr output: mini_pupper_driver`.
  * A. The following warnings can be safely ignored. See [mini_pupper_ros#45](https://github.com/mangdangroboticsclub/mini_pupper_ros/pull/45#discussion_r1104759104) for details.
  ```
  Starting >>> mini_pupper_description
  --- stderr: mini_pupper_driver
  /usr/lib/python3/dist-packages/setuptools/command/easy_install.py:158: EasyInstallDeprecationWarning: easy_install command is deprecated. Use build and pip and other standards-based tools.
    warnings.warn(
  /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
    warnings.warn(
  /usr/lib/python3/dist-packages/pkg_resources/__init__.py:116: PkgResourcesDeprecationWarning: 1.1build1 is an invalid version and will not be supported in a future release
    warnings.warn(
  /usr/lib/python3/dist-packages/pkg_resources/__init__.py:116: PkgResourcesDeprecationWarning: 0.1.43ubuntu1 is an invalid version and will not be supported in a future release
    warnings.warn(
  ---
  Finished <<< mini_pupper_driver [7.37s]
  ```
