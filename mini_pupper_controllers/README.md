# mini_pupper_controllers

Custom ros2_control controllers for Mini Pupper 2.

## Controllers

### SimpleQuadrupedController

A position controller that:
- Holds joints at configured `default_positions` on activation
- Accepts position commands via ROS 2 topic to override the default pose
- Falls back to `default_positions` when no external command is active

**Type**: `mini_pupper_controllers/SimpleQuadrupedController`

**Parameters**:
- `joints` (string_array): Joint names in the same order as the hardware interface
- `default_positions` (double_array): Positions in radians to hold on startup. Knee values must be `knee_abs = hip + knee`, not the raw joint angle.

**Subscribed Topics**:
- `~/commands` (`std_msgs/Float64MultiArray`): 12-element array of joint position commands (rad)

**Example configuration** (from `mini_pupper_2_controllers.yaml`):
```yaml
simple_quadruped_controller:
  ros__parameters:
    joints:
      - base_lf1
      - lf1_lf2
      - lf2_lf3
      - base_rf1
      - rf1_rf2
      - rf2_rf3
      - base_lb1
      - lb1_lb2
      - lb2_lb3
      - base_rb1
      - rb1_rb2
      - rb2_rb3
    default_positions:
      - -0.080   # base_lf1  (abduction)
      - 1.078    # lf1_lf2   (hip)
      - -0.905   # lf2_lf3   (knee_abs = 1.078 + (-1.983))
      - 0.080    # base_rf1
      - 1.078    # rf1_rf2
      - -0.905   # rf2_rf3
      - -0.080   # base_lb1
      - 1.078    # lb1_lb2
      - -0.905   # lb2_lb3
      - 0.080    # base_rb1
      - 1.078    # rb1_rb2
      - -0.905   # rb2_rb3
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_controllers
source install/setup.bash
```

## License

Apache-2.0
