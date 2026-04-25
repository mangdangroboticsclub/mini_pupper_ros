# mini_pupper_hardware

ROS 2 `ros2_control` hardware interface plugin for the Mini Pupper 2 quadruped robot.

## Architecture

This package integrates Mini Pupper 2 with [ros2_control](https://control.ros.org) — the industry-standard hardware abstraction framework for ROS 2. ros2_control decouples controllers from hardware: controllers talk to abstract command/state interfaces, while the hardware plugin handles the actual servo protocol. This enables controller hot-swapping at runtime, a fixed-rate real-time hardware loop, and full simulation/hardware parity via a mock plugin — all without changing any controller code.

```
ros2_control framework
    ↓  (position commands / state feedback)
MiniPupperHardware  (C++ SystemInterface plugin)
    ↓  (BB12B12H socket protocol, 38 bytes)
ESP32Interface  (C++ socket client)
    ↓  (Unix domain socket  /tmp/esp32-proxy.socket)
esp32-proxy  (running on robot)
    ↓
12× servo motors
```

## Socket protocol (BB12B12H)

38-byte packet sent to `/tmp/esp32-proxy.socket` (`SOCK_SEQPACKET`):

| Bytes | Format | Content |
|-------|--------|---------|
| 0 | `B` | Packet size = 38 |
| 1 | `B` | Command = 1 (SETPOS) |
| 2–13 | `12B` | Torque enable per servo (uint8, 1 = enabled) |
| 14–37 | `12H` | Servo positions, uint16 little-endian (0–1023, neutral = 512) |

## Servo layout

| Index | Joint | Leg |
|-------|-------|-----|
| 0–2 | abd, hip, knee_abs | RF (right front) |
| 3–5 | abd, hip, knee_abs | LF (left front) |
| 6–8 | abd, hip, knee_abs | RB (right back) |
| 9–11 | abd, hip, knee_abs | LB (left back) |

`knee_abs = hip_angle + knee_angle` (absolute, not relative to hip).

## Angle → servo conversion

```
servo = 512 − MICROS_PER_RAD × (angle − neutral_angle) × multiplier

MICROS_PER_RAD = (760 − 210) / π  ≈ 175.07
```

Neutral angles and per-axis/per-leg multipliers are defined in
`include/mini_pupper_hardware/mini_pupper_hardware.hpp`.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select mini_pupper_hardware
source install/setup.bash
```

## Running

```bash
ros2 launch mini_pupper_bringup bringup_with_stanford_controller.launch.py
```

Set `hardware_interface_type: mock` in the URDF `<ros2_control>` block to run
without a physical robot.

## Hardware diagnostic tests

The `test/` directory contains standalone diagnostic scripts that run directly
on the robot without ROS 2. See [`test/README.md`](test/README.md).

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Plugin not found | `colcon build`, then `source install/setup.bash` |
| Servos not moving | Is `esp32-proxy` running? Does `/tmp/esp32-proxy.socket` exist? |
| Wrong standing pose | Check `default_positions` in `mini_pupper_2_controllers.yaml` — knee values must be `knee_abs` (absolute angle from IK ≈ −0.774 at `default_z_ref=-0.07`), not the raw inter-link angle |
| Front/rear leg mismatch | Check `SERVO_MULTIPLIERS` abduction row in `mini_pupper_hardware.hpp` |

## License

Apache License 2.0
