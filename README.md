[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Humble-brightgreen)](http://docs.ros.org/en/humble/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-22.04-green)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2/LICENSE)
&nbsp;
[![Twitter URL](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Ftwitter.com%2FLeggedRobot)](https://twitter.com/LeggedRobot)

# Mini Pupper ROS 2 Humble

Make sure the bsp package is installed to your pupper before installing this repositiory if you are not using a pre-built image to install this repository. 
Please refer to https://github.com/mangdangroboticsclub/mini_pupper_2_bsp (bsp for mini pupper 2) or https://github.com/mangdangroboticsclub/mini_pupper_bsp (bsp for mini pupper) according to your model.

For a more detailed guide, please refer to our documentation website https://minipupperdocs.readthedocs.io/en/latest/.

Supported versions

* Ubuntu 22.04 + ROS 2 Humble

## 1. Installation

__Please note that the setup of PC and the mini pupper is separated__

* To use Gazebo simulator, "1.1 PC Setup" is required.
* To control Mini Pupper, "1.2 Mini Pupper Setup" is required.
* To control Mini Pupper using visualize tools, "1.1 PC Setup" and "1.2 Mini Pupper Setup" is required.
* To use Gazebo simulator or control Mini Pupper using joystick,  "1.4 Joystick Setup" is required.

### 1.1 PC Setup 

PC Setup corresponds to PC (your desktop or laptop PC) for controlling Mini Pupper remotely or execute simulator.  
__Do not apply these PC Setup commands to your Raspberry Pi on Mini Pupper.__

Ubuntu 22.04 + ROS 2 Humble is required.  

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
vcs import < mini_pupper_ros/.minipupper.repos --recursive
```

Build and install all ROS packages.

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-teleop-twist-keyboard
sudo apt install ros-humble-teleop-twist-joy
pip3 install simple_pid
colcon build --symlink-install
```

Reference(Just for reference, don't need to do it again.): 

[installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or

[unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)


### 1.2 Mini Pupper Setup

Mini Pupper Setup corresponds to the Raspberry Pi on your Mini Pupper.  
Ubuntu 22.04 is required.

Before installation, you need to install the BSP(board support package) repo for your [Mini Pupper 2](https://github.com/mangdangroboticsclub/mini_pupper_2_bsp) or [Mini Pupper](https://github.com/mangdangroboticsclub/mini_pupper_bsp.git).

After installing the driver software, install ROS 2 Humble is required.  

```sh
cd ~
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev mini_pupper_ros
cd mini_pupper_ros
./install.sh
```

Reference(Just for reference, don't need to do it again.): 

[installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or

[unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu)


### 1.3 Connecting Mini Pupper to PC

The mini pupper is assumed to be automatically connected to the PC under same internet environment, but to make sure the mini pupper is connected to the PC, we can check their ROS domain ID (which can be defined by user and is 0 in default cases) according to the following steps:

```sh
# Terminal 1 (ssh to real mini pupper)
export | grep "ROS_DOMAIN_ID"
```

```sh
# Terminal 2 (on PC)
export | grep "ROS_DOMAIN_ID"
```

Compare the output and check if they are the same in both terminals:

Example output:

![check_ROS_DOMAIN_ID.png](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ab89763e788d413c3aafd302cad849a417732fe6/imgs/check_ROS_DOMAIN_ID.png)

If the __ID are different in both terminal or there is no output__ of the above commnand, you will have to set the ROS_DOMAIN_ID to the same number using the following command (which number is used does not matter):

To tackle the example output, we can use the following command to set the same id on both terminal:

__This command can be used on both PC and Mini Pupper__

```sh
# Terminal 1 (ssh to real mini pupper)
export ROS_DOMAIN_ID=42
```

```sh
# Terminal 2 (on PC)
export ROS_DOMAIN_ID=42
```

Use the following command in both terminals to confirm that the PC and the mini pupper are connected:

```sh
# Terminal 1 (ssh to real mini pupper)
ros2 node list
```

```sh
# Terminal 2 (on PC)
ros2 node list
```

Compare the output in both terminals:

![nodeList.png](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ab89763e788d413c3aafd302cad849a417732fe6/imgs/nodeList.png)

If the output in __both terminals shows the same list of node__ which is similar to the picture, your PC and the mini pupper is connected. The following steps can be proceeded.
__Note that the node list depends on the nodes in progress, which may not be exactly the same from the image.__

### 1.4 Joystick Setup

Press the HOME button on the controller. Then search for available bluetooth devices on your PC.

- To check whether the joystick is connected
```sh
#(on PC)
sudo apt install joystick 
jstest /dev/input/js0 # there will be output once the joystick is connected
```

## 2. Quick Start

## 2.1 PC
### 2.1.1 Test in RViz

Note: This step is only for PC

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=False
```
```sh
# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup rviz.launch.py
```

- If using keyboard control
```sh
# Terminal 3
. ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- If using joystick control
```sh
# Terminal 3
. ~/ros2_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py
```

### 2.1.2 Test in Gazebo

Note: This step is only for PC

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_gazebo gazebo.launch.py
```

- If using keyboard control
```sh
# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- If using joystick control
```sh
# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py
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
ros2 launch mini_pupper_slam slam.launch.py use_sim_time:=true
```

- If using keyboard control
```sh
# Terminal 3
. ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- If using joystick control
```sh
# Terminal 3
. ~/ros2_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py
```

- Save the map  
```sh
# Terminal 4 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
The map will be saved under home directory. Two files will be generated, namely map.pgm and map.yaml.


#### 2.1.4 Test Navigation in Gazebo

- Bring up Gazebo
```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_gazebo gazebo.launch.py
```

- Navigation   
```sh
# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation navigation.launch.py use_sim_time:=true
```
Alternatively, if you wish to use the map you generated in previous step, you can specify the map path with the following command.
```sh
ros2 launch mini_pupper_navigation navigation.launch.py use_sim_time:=true map:=$HOME/map.yaml
```
After executing this command, the rviz window will be displayed. You can utilize the "2D Post Estimate" feature to establish an appropriate initial pose for the robot. Subsequently, you can set a goal for the robot by clicking on "Nav2 Goal", hence Nav2 will plan the path and guide the robot towards reaching the goal.


## 2.2 Mini Pupper

### 2.2.1 Test walk

Note: This step is only for Mini Pupper

Open 2 terminals and ssh login to Mini Pupper on both.

```sh
# Terminal 1 (ssh)
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

- If using keyboard control
```sh
# Terminal 2 (ssh or PC)
. ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- If using joystick control
```sh
# Terminal 2 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py
```

### 2.2.2 Test SLAM (Mapping)

Note: This step requires both PC and Mini Pupper

- Bring up real mini pupper
```sh
# Terminal 1 (ssh to real mini pupper)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

- SLAM on PC
```sh
# Terminal 2 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_slam slam.launch.py
```

Remotely control the Mini Pupper to complete the mapping.
- If using keyboard control
```sh
# Terminal 3 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
- If using joystick control
```sh
# Terminal 3 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch teleop_twist_joy teleop-launch.py
```

- Save the map  
Run the following command from a new terminal.
```sh
# Terminal 4 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
The map will be saved under home directory. Two files will be generated, namely map.pgm and map.yaml.

#### 2.2.3 Test Navigation

- Bring up real mini pupper
```sh
# Terminal 1 (ssh to real mini pupper)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py

```

- Navigation with previously saved map from step 2.2.2
```sh
# Terminal 4 (on PC)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation navigation.launch.py map:=$HOME/map.yaml
```

### 2.2.4 Test dance
Please refer to the README.md inside package "mini_pupper_dance".

### 2.2.5 Image Recognition

This package base on camera, and is tested utilising a package called v4l2 camera that focuses on Raspberry Pi Camera for most systems.

Note: This step can be done by only using Mini Pupper or both PC and Mini Pupper

Before bringing up mini pupper, please change the config file under
```
~/ros2_ws/src/mini_pupper_ros/mini_pupper_bringup/config
```
Open the configuration file according to the model that you are using (eg. if you are using mini pupper 2 then change the value of mini_pupper_2.yaml). 

Change the value of camera to true and that of lidar to false so that camera is turned on for the function as the following example of mini pupper 2.

```yaml
sensors:
  lidar: false
  imu: true
  camera: true
ports: 
  lidar: '/dev/ttyAMA1'
```

- Bring up real mini pupper
```sh
# Terminal 1 (ssh to real mini pupper)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

- Launch all nodes (currently including line following only)
```sh
# Terminal 2 (ssh or PC)
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_recognition recognition.launch.py #pid:=false if want to disable pid
```

- Launch the GenAI Image Recognition Line Following Demo (This is a code sample written to demonstrate the use of GenAI for image recognition and line following, but it is not recommended to use it as a primary function for line following, as there may be significant delays) This code requires access to our repository of generative AI models and supporting infrastructure.
```sh
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_recognition cloud_line_demo.launch.py
```

## License

```
Copyright 2022-2024 MangDang

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
