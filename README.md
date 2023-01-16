# Mini Pupper ROS 2

## 1. Installation

* To use Gazebo simulator, "1.1 PC Setup" is required.
* To control Mini Pupper, "1.2 Mini Pupper Setup" is required.
* To control Mini Pupper using visualize tools, "1.1 PC Setup" and "1.2 Mini Pupper Setup" is required.

### 1.1 PC Setup

PC Setup corresponds to PC (your desktop or laptop PC) for controlling Mini Pupper remotely or execute simulator.  
__Do not apply these commands to your Mini Pupper.__

Ubuntu 22.04 + ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

```sh
cd ~
mkdir -p ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
vcs import < minipupper_ros/.minipupper.repos --recursive
```

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-humble-teleop-twist-keyboard
colcon build --symlink-install
```

### 1.2 Mini Pupper Setup

Mini Pupper Setup corresponds to the Raspberry Pi on your Mini Pupper.

Ubuntu 22.04 + ROS 2 Humble is required.  
Please follow the [installation document for ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the [unofficial ROS 2 installation script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu).

```sh
cd ~
mkdir -p ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
vcs import < minipupper_ros/.minipupper.repos --recursive
touch champ/champ/champ_description/AMENT_IGNORE
touch champ/champ/champ_gazebo/AMENT_IGNORE
touch champ/champ/champ_navigation/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_gazebo/AMENT_IGNORE
touch mini_pupper_ros/mini_pupper_navigation/AMENT_IGNORE
```

```sh
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys=joint_state_publisher_gui --skip-keys=rviz2
sudo apt-get install ros-humble-teleop-twist-keyboard
colcon build --symlink-install
```

## 2. Quick Start

### 2.1 Test in RViz

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py rviz:=true

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```

### 2.2 Test in Gazebo

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_gazebo gazebo.launch.py rviz:=true

# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```

### 2.3 Cartographer Test in Gazebo

```sh
# Terminal 1
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_gazebo gazebo.launch.py

# Terminal 2
. ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_navigation slam.launch.py use_sim_time:=true

# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```
