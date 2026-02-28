[![ROS2 VERSION](https://img.shields.io/badge/ROS-ROS%202%20Jazzy-brightgreen)](http://docs.ros.org/en/jazzy/index.html)
&nbsp;
[![Ubuntu VERSION](https://img.shields.io/badge/Ubuntu-24.04-green)](https://ubuntu.com/)
&nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/mini_pupper_ros/blob/ros2/LICENSE)
&nbsp;
[![Build Status](https://github.com/mangdangroboticsclub/mini_pupper_ros/actions/workflows/industrial_ci.yml/badge.svg)](https://github.com/mangdangroboticsclub/mini_pupper_ros/actions/workflows/industrial_ci.yml)
&nbsp;
[![Docs](https://img.shields.io/badge/docs-latest-blue)](https://minipupperdocs.readthedocs.io/en/latest/)
&nbsp;
[![Twitter URL](https://img.shields.io/twitter/url?style=social&url=https%3A%2F%2Ftwitter.com%2FLeggedRobot)](https://twitter.com/LeggedRobot)

# Mini Pupper ROS 2 Jazzy

<p align="left">
  <img src="imgs/mini_pupper_2.jpg" alt="Mini Pupper 2" width="480"/>
</p>

A comprehensive ROS 2 robotics platform for autonomous navigation, computer vision, and multi-robot coordination. Built for research, education, and development on **Ubuntu 24.04** with **ROS 2 Jazzy**.

> **Migration Notice**: This branch has been migrated from ROS 2 Humble to ROS 2 Jazzy. Gazebo Classic has been replaced with **Gazebo Sim (Harmonic)** using `ros_gz_sim`, `ros_gz_bridge`, and `gz_ros2_control`.

## Key Capabilities

<p align="left">
  <img src="imgs/mini_pupper_navigation_480_12.gif" alt="Navigation Demo" width="480"/>
</p>

### SLAM & Autonomous Navigation
Real-time mapping and path planning with ROS 2 Nav2 stack integration.  
**[Documentation](mini_pupper_navigation/README.md)**

### Computer Vision Tracking
YOLO11-powered person detection and following with multi-object tracking.  
**[Documentation](mini_pupper_tracking/README.md)**

### Multi-Robot Fleet Management
Centralized command and control for coordinated multi-robot operations.  
**[Documentation](mini_pupper_fleet/README.md)**

### Choreographed Movement System
Programmable dance sequences with audio synchronization capabilities.  
**[Documentation](mini_pupper_dance/README.md)**

## Quick Start

### System Requirements
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**: Jazzy Jalisco
- **Simulator**: Gazebo Sim (Harmonic) - replaces Gazebo Classic

### Installation

#### Option 1: Automated Install (Recommended)
```bash
# Clone the repository
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev
cd mini_pupper_ros

# Run the installer
./pc_install.sh  # For PC/simulation
# OR
./pupper_install.sh  # For Mini Pupper hardware
```

#### Option 2: Manual Setup
```bash
# Install ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2-dev

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
```

### Running Simulation (Gazebo Sim)
```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch simulation with Gazebo Sim (Harmonic)
ros2 launch mini_pupper_simulation main.launch.py

# In another terminal - keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Test Basic Movement (Hardware)
```bash
# Terminal 1 (SSH to robot)
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py
```
```bash
# Terminal 2 (PC or robot)
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Migration from Humble

If you're migrating from ROS 2 Humble:
- Update to **Ubuntu 24.04** (required for Jazzy)
- Gazebo Classic is replaced with **Gazebo Sim (Harmonic)**
- Topic mappings are handled automatically via `ros_gz_bridge`
- All launch files updated to use `ros_gz_sim` instead of `gazebo_ros`

## Contributing
Contributions are welcome.
Whether it's reporting a bug, suggesting a new feature, or submitting a pull request, we'd love your help in improving the **Mini Pupper ROS 2 project**.  

- Read our [CONTRIBUTING.md](CONTRIBUTING.md) guide to get started.  
- Explore or work on [open issues](https://github.com/mangdangroboticsclub/mini_pupper_ros/issues).  

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
