# mini_pupper_controllers

Custom ros2_control controllers for Mini Pupper robot.

## Overview

This package provides custom controller implementations for the Mini Pupper quadruped robot using the ros2_control framework. These controllers can be used both in simulation (Gazebo) and on real hardware.

## Controllers

### SimpleQuadrupedController

A basic position controller that:
- Holds joints at configured default positions
- Accepts position commands via ROS 2 topic
- Provides smooth transitions between commanded positions

**Type**: `mini_pupper_controllers/SimpleQuadrupedController`

**Parameters**:
- `joints` (string_array): List of joint names to control
- `default_positions` (double_array): Default positions for each joint (in radians)

**Subscribed Topics**:
- `~/commands` (std_msgs/Float64MultiArray): Joint position commands

**Example Configuration**:
```yaml
simple_quadruped_controller:
  ros__parameters:
    type: mini_pupper_controllers/SimpleQuadrupedController
    joints:
      - front_left_shoulder_joint
      - front_left_leg_joint
      - front_left_foot_joint
      - front_right_shoulder_joint
      - front_right_leg_joint
      - front_right_foot_joint
      - rear_left_shoulder_joint
      - rear_left_leg_joint
      - rear_left_foot_joint
      - rear_right_shoulder_joint
      - rear_right_leg_joint
      - rear_right_foot_joint
    default_positions: [0.0, 0.785, -1.57, 0.0, 0.785, -1.57, 0.0, 0.785, -1.57, 0.0, 0.785, -1.57]
```

## Building

This package is part of the mini_pupper_ros workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_controllers
source install/setup.bash
```

## Usage

Load the controller using the controller_manager spawner:

```bash
ros2 run controller_manager spawner simple_quadruped_controller
```

## License

Apache-2.0
