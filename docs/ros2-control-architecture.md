# ROS 2 Control Architecture

This document describes the `ros2_control` stack used by Mini Pupper 2 — how the hardware interface, controller, and controller manager fit together, and how the setup differs between the real robot and simulation.

---

## Overview

```
┌──────────────────────────────────────────────────────────────────────┐
│                        controller_manager                            │
│  (ros2_control_node @ 100 Hz)                                        │
│                                                                      │
│  ┌─────────────────────────────┐  ┌──────────────────────────────┐   │
│  │  joint_state_broadcaster    │  │  simple_quadruped_controller │   │
│  │  (reads state_interfaces)   │  │  (writes command_interfaces) │   │
│  └────────────┬────────────────┘  └──────────────┬───────────────┘   │
│               │ /joint_states                     │ ~/commands        │
│               │                                   │ (Float64MultiArray│
│               │                                   │  12 joint pos)    │
└───────────────┼───────────────────────────────────┼───────────────────┘
                │                                   │
                │      hardware_interface           │
                │  ┌────────────────────────────┐   │
                │  │   MiniPupperHardware  /     │   │
                └──┤   GazeboSystem (sim)        ├───┘
                   └──────────────┬──────────────┘
                                  │
                    Real: ESP32 proxy (Unix socket)
                    Sim:  Gazebo joint actuators
```

---

## Joints

Mini Pupper 2 has **12 joints** — four legs with three joints each:

| Index | Joint name  | Axis       | Leg          |
|-------|-------------|------------|--------------|
| 0     | base_lf1    | abduction  | Left Front   |
| 1     | lf1_lf2     | hip        | Left Front   |
| 2     | lf2_lf3     | knee       | Left Front   |
| 3     | base_rf1    | abduction  | Right Front  |
| 4     | rf1_rf2     | hip        | Right Front  |
| 5     | rf2_rf3     | knee       | Right Front  |
| 6     | base_lb1    | abduction  | Left Back    |
| 7     | lb1_lb2     | hip        | Left Back    |
| 8     | lb2_lb3     | knee       | Left Back    |
| 9     | base_rb1    | abduction  | Right Back   |
| 10    | rb1_rb2     | hip        | Right Back   |
| 11    | rb2_rb3     | knee       | Right Back   |

All joints expose a **`position` state interface** and a **`position` command interface** (real robot) or **`position_pid` command interface** (simulation).

---

## Hardware Interface — `MiniPupperHardware`

**Package:** `mini_pupper_hardware`  
**Plugin:** `mini_pupper_hardware/MiniPupperHardware`  
**Base class:** `hardware_interface::SystemInterface`

### Modes

| Mode | Activated when | Behaviour |
|------|----------------|-----------|
| `esp32_proxy` | Real robot (`hardware_interface_type: esp32_proxy`) | Communicates with the ESP32 proxy daemon via Unix socket `/tmp/esp32-proxy.socket` |
| `mock` | Default / no hardware | Mirrors position commands directly to state; useful for software-only testing |

### Read / Write rates

The control loop runs at **100 Hz**. Because blocking serial reads would stall the loop, hardware state is read only every 10 cycles (**~10 Hz**). Commands are sent every cycle at full rate.

### Servo conversion

The real servo uses the legacy MangDang servo model:

```
servo_value = NEUTRAL_POSITION + MICROS_PER_RAD × (angle − neutral_angle) × multiplier
```

Constants:
- `NEUTRAL_POSITION = 512`
- `MICROS_PER_RAD = (760 − 210) / π ≈ 175`
- `NEUTRAL_ANGLES_RAD = [0, π/4, −π/4]` (abduction, hip, knee)
- Per-axis, per-leg sign multipliers handle the mirrored kinematics between left/right and front/back legs.

The hardware expects joints in the order **RF, LF, RB, LB** (right-front first), while `ros2_control` stores them in URDF order **LF, RF, LB, RB**. A `hardware_joint_to_urdf_index_` mapping is built at `on_init` to translate between the two.

### Parallel linkage (knee joint)

On the real robot the knee servo is mechanically driven by a parallel 4-bar linkage so the commanded angle is the **absolute** thigh+knee angle:

```
servo_knee_cmd = hip_angle + knee_angle
```

In the URDF the knee joint is modelled as a **serial joint** relative to the thigh. When running in simulation, the `SimpleQuadrupedController` compensates for this before writing to command interfaces (see below).

---

## Controller — `SimpleQuadrupedController`

**Package:** `mini_pupper_controllers`  
**Plugin:** `mini_pupper_controllers/SimpleQuadrupedController`  
**Base class:** `controller_interface::ControllerInterface`

### Subscription

```
~/commands  →  std_msgs/msg/Float64MultiArray
```

The `data` field must contain exactly **12 values** in joint order (LF, RF, LB, RB × abd, hip, knee). The Stanford controller publishes to this topic.

### Update logic (100 Hz)

```
1. Read latest command from lock-free realtime buffer
2. If no external command received yet:
     a. If idle_ramp_enabled: hold measured pose, then smoothly ramp to default_positions
     b. Otherwise:          hold default_positions immediately
3. If external command received:
     a. (simulation only) apply_linkage_compensation()
     b. Write 12 position values to command_interfaces
```

### Parallel linkage compensation (simulation only)

When `parallel_linkage_compensation: true` (set automatically in the simulation YAML), the controller inverts the 4-bar model so Gazebo receives the URDF-space serial angle:

```
urdf_knee_cmd = knee_cmd − hip_cmd
```

This is applied per-leg before writing to `command_interfaces`.

### Idle ramp (simulation only)

When `idle_ramp_enabled: true` the controller:

1. Holds the measured active pose for `idle_hold_duration_sec` (3 s by default) after activation.
2. Then smoothly ramps using a **smoothstep** (S-curve) interpolation over `idle_ramp_duration_sec` (5 s by default) to the configured `default_positions`.

This prevents the robot from jumping to the stand posture right after Gazebo spawns.

---

## Controller Manager

### Real robot

Launched by `robot_ros2_controllers.launch.py`:

```
controller_manager_node starts
    └─ OnProcessStart → joint_state_broadcaster spawner
         └─ OnProcessExit → simple_quadruped_controller spawner
```

Config file: `mini_pupper_bringup/config/ros2_control/mini_pupper_2_controllers.yaml`  
Update rate: **100 Hz**, `use_sim_time: false`

### Simulation

Launched by the `gazebo_ros2_control` plugin embedded in the URDF. The plugin loads the controller YAML and spawns both controllers via `sim_ros2_controllers.launch.py`.

Config file: `mini_pupper_simulation/config/ros2_control/mini_pupper_2_controllers_sim.yaml`  
Update rate: **100 Hz**, `use_sim_time: true`

---

## URDF / Xacro

The ros2_control block is defined in `mini_pupper_description/urdf/ros2_control/mini_pupper.ros2_control.xacro`.

```xml
<!-- Real robot -->
<plugin>mini_pupper_hardware/MiniPupperHardware</plugin>
<param name="hardware_interface_type">esp32_proxy</param>

<!-- Simulation (Gazebo) -->
<plugin>gazebo_ros2_control/GazeboSystem</plugin>
```

Command interface type also differs:

| Environment | Command interface | Notes |
|-------------|-------------------|-------|
| Real robot  | `position`        | Direct position setpoint to hardware |
| Simulation  | `position_pid`    | Gazebo PID (kp=5.0, kd=0.05) tuned for 100× inertia scaling |

---

## Data flow summary

### Real robot

```
stanford_controller
    │ /simple_quadruped_controller/commands (Float64MultiArray, 12 joints)
    ▼
SimpleQuadrupedController::update()
    │ command_interfaces[0..11]/position
    ▼
MiniPupperHardware::write()
    │ servo_value = f(joint_angle, multipliers, neutral)
    ▼
ESP32Interface → Unix socket → esp32-proxy daemon → physical servos

Physical servos → esp32-proxy → ESP32Interface
    │ (throttled: 1 read per 10 write cycles ≈ 10 Hz)
    ▼
MiniPupperHardware::read()
    │ state_interfaces[0..11]/position
    ▼
JointStateBroadcaster → /joint_states
```

### Simulation

```
stanford_controller
    │ /simple_quadruped_controller/commands (Float64MultiArray, 12 joints)
    ▼
SimpleQuadrupedController::update()
    │ apply_linkage_compensation()  ← serial URDF ≠ parallel hardware model
    │ command_interfaces[0..11]/position_pid
    ▼
GazeboSystem → Gazebo joint actuators (Gazebo handles PID internally)

Gazebo joint sensors
    │ state_interfaces[0..11]/position
    ▼
JointStateBroadcaster → /joint_states
```

---

## Configuration reference

| File | Purpose |
|------|---------|
| `mini_pupper_bringup/config/ros2_control/mini_pupper_2_controllers.yaml` | Real-robot controller manager + controller params |
| `mini_pupper_simulation/config/ros2_control/mini_pupper_2_controllers_sim.yaml` | Simulation controller manager + controller params (includes idle ramp, linkage compensation) |
| `mini_pupper_description/urdf/ros2_control/mini_pupper.ros2_control.xacro` | Hardware plugin selection, joint definitions, initial values |

---

## Key differences: real robot vs simulation

| Aspect | Real robot | Simulation |
|--------|-----------|------------|
| Hardware plugin | `MiniPupperHardware` (ESP32 proxy) | `GazeboSystem` |
| Command interface | `position` | `position_pid` (kp=5, kd=0.05) |
| Knee angle | Hardware-space absolute (hip+knee) | URDF-space serial (linkage compensation applied by controller) |
| Idle ramp | Disabled | Enabled (3 s hold + 5 s S-curve ramp) |
| `use_sim_time` | false | true |
| Controller spawning | Event-driven (OnProcessStart / OnProcessExit) | `gazebo_ros2_control` plugin + `sim_ros2_controllers.launch.py` |
