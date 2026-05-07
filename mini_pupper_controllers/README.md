# mini_pupper_controllers

Custom ros2_control controllers for Mini Pupper 2.

## Controllers

### SimpleQuadrupedController

A position controller that:
- Accepts position commands via ROS 2 topic
- Before the first external command, holds the current measured pose by default
- Can optionally ramp from the startup pose to configured `default_positions`
- Applies simulation-only knee linkage compensation when enabled

**Type**: `mini_pupper_controllers/SimpleQuadrupedController`

**Parameters**:
- `joints` (string_array): Joint names in the same order as the hardware interface
- `default_positions` (double_array): Target joint positions in radians for the idle pose. In hardware config, knee values match the Stanford IK output. In simulation config, values are the URDF joint angles after linkage compensation.
- `parallel_linkage_compensation` (bool): Converts Stanford hardware-space knee commands to the URDF knee convention used in simulation
- `idle_ramp_enabled` (bool): Enables gradual motion from the measured startup pose to `default_positions` before any external command arrives
- `idle_hold_duration_sec` (double): How long to hold the startup pose before beginning the idle ramp
- `idle_ramp_duration_sec` (double): Ramp duration from startup pose to `default_positions`

**Subscribed Topics**:
- `~/commands` (`std_msgs/Float64MultiArray`): 12-element array of joint position commands (rad)

## License

Apache-2.0
