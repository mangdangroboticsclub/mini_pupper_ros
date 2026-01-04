# Mini Pupper ROS 2 Hardware Interface

This package provides a ros2_control hardware interface for the Mini Pupper quadruped robot. It integrates with the MangDang Mini Pupper hardware stack and enables control via the ros2_control framework.

## Architecture Overview

The hardware interface works by bridging between:

1. **ros2_control Framework**: Standard ROS 2 control interface for managing joint commands and state feedback
2. **MangDang Hardware Stack**: Python-based hardware interface in `mini_pupper_2_bsp/Python_Module/MangDang/mini_pupper`
3. **Hardware**: ESP32-based servo control system (via esp32_proxy or direct communication)

```
ros2_control
    ↓
MiniPupperHardware (C++ plugin)
    ↓
Python Wrapper (py_hardware_interface.py)
    ↓
MangDang HardwareInterface
    ↓
ESP32 / Hardware
```

## File Structure

```
mini_pupper_hardware/
├── include/
│   └── mini_pupper_hardware/
│       └── mini_pupper_hardware.hpp      # Header file with hardware interface class
├── src/
│   └── mini_pupper_hardware.cpp          # Implementation of hardware interface
├── mini_pupper_hardware/
│   ├── __init__.py                       # Python module init
│   └── py_hardware_interface.py          # Python wrapper for hardware communication
├── config/
│   ├── mini_pupper_system.urdf.xacro   # URDF definition with ros2_control config
│   └── controllers.yaml                  # Controller configuration
├── launch/
│   └── mini_pupper_hardware.launch.py   # Launch file
├── test/
│   └── test_mini_pupper_hardware.cpp    # Unit tests
├── mini_pupper_hardware_plugins.xml      # Plugin export configuration
├── package.xml                           # ROS 2 package manifest
└── CMakeLists.txt                        # Build configuration
```

## Joint Naming Convention

The Mini Pupper has 12 joints (3 per leg, 4 legs):

- **Right Front**: rf1, rf2, rf3 (abduction, inner hip, outer hip)
- **Left Front**: lf1, lf2, lf3 (abduction, inner hip, outer hip)
- **Right Back**: rb1, rb2, rb3 (abduction, inner hip, outer hip)
- **Left Back**: lb1, lb2, lb3 (abduction, inner hip, outer hip)

## Configuration

### Hardware Interface Type

In `config/mini_pupper_system.urdf.xacro`, you can configure the hardware interface:

```xml
<param name="hardware_interface_type">mock</param>
```

Options:
- `"mock"`: Simulation mode, no real hardware communication
- `"esp32_proxy"`: Use ESP32 proxy for real hardware control
- `"direct"`: Direct ESP32 communication (if available)

### Controllers

The default controller is `joint_trajectory_controller` configured in `config/controllers.yaml`:

- Accepts joint trajectory messages
- Executes smooth trajectories to target positions
- Provides feedback on joint state (position, velocity, effort)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_hardware
source install/setup.bash
```

## Running

### Launch with default (mock) hardware:

```bash
ros2 launch mini_pupper_hardware mini_pupper_hardware.launch.py
```

### Send joint trajectory commands:

```bash
# Example: Move all joints to neutral position
ros2 topic pub -1 /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"header: {stamp: now, frame_id: ''} \
joint_names: [rf1, rf2, rf3, lf1, lf2, lf3, rb1, rb2, rb3, lb1, lb2, lb3] \
points: [{positions: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], time_from_start: {sec: 1, nanosec: 0}}]"
```

## Replacing the Old Servo Interface

Previously, the system used `servo_interface.py` which:
- Subscribed directly to `/joint_group_effort_controller/joint_trajectory`
- Manually converted joint angles to hardware format
- Directly called hardware methods

The new ros2_control approach:
- Uses standard ros2_control interfaces
- Automatically handles controller lifecycle management
- Supports multiple controllers (trajectory, effort, velocity, etc.)
- Provides better separation of concerns
- Follows ROS 2 standards

### Migration Steps

1. **Update URDF**: Include the `mini_pupper_system.urdf.xacro` config instead of direct servo setup
2. **Replace servo_interface**: Use ros2_control and the new hardware interface instead
3. **Use standard controllers**: Use `joint_trajectory_controller` or other standard ros2_control controllers
4. **Update launch files**: Point to the new hardware interface launch file

## Python Hardware Interface Wrapper

The `py_hardware_interface.py` module provides:

```python
from mini_pupper_hardware.py_hardware_interface import MiniPupperHardwareInterface

# Initialize (mock mode for testing)
hw = MiniPupperHardwareInterface(use_mock=True)

# Set joint positions (12 floats)
positions = [0.0] * 12
hw.set_joint_positions(positions)

# Get current positions
current = hw.get_joint_positions()

# Cleanup
hw.shutdown()
```

## Integration with MangDang Hardware

The hardware interface expects the MangDang mini_pupper Python module to be installed:

```bash
cd ~/projects/mini_pupper_2_bsp/Python_Module
pip install -e .
```

The wrapper then uses:
```python
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
```

## Testing

Run unit tests:

```bash
colcon test --packages-select mini_pupper_hardware
```

## Future Enhancements

1. **Actual Hardware Feedback**: Implement `read_state_from_hardware()` to get real feedback
2. **Effort Control**: Add effort/torque feedback and control
3. **IMU Integration**: Add IMU state to the interface
4. **Battery Monitoring**: Include battery voltage feedback
5. **Calibration Interface**: Provide calibration configuration support

## Troubleshooting

### Plugin not found
- Rebuild: `colcon build --packages-select mini_pupper_hardware`
- Source setup: `source install/setup.bash`

### Hardware not responding
- Check ESP32 connection
- Verify `hardware_interface_type` parameter
- Check MangDang module installation

### Joints not moving
- Verify URDF config in `mini_pupper_system.urdf.xacro`
- Check controller status: `ros2 service call /controller_manager/list_controllers`
- Inspect joint trajectory controller logs

## References

- [ros2_control Documentation](https://control.ros.org/)
- [Hardware Interface API](https://github.com/ros-controls/ros2_control)
- [Mini Pupper Project](https://github.com/MangDang/mini_pupper)

## License

Apache License 2.0
