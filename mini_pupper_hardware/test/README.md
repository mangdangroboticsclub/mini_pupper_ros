# Hardware Diagnostic Tests

These are **manual diagnostic tests** to be run directly on the robot.
They are **not** unit tests and are **not** executed by `colcon test`.

## Files

| File | Language | Purpose |
|------|----------|---------|
| `standalone_hardware_test.cpp` | C++ | Validates servo math and live hardware without ROS 2 |
| `standalone_hardware_test.py` | Python | Same tests using the MangDang Python library directly |

Both tests have two modes:

- **Math-only** (default): verifies `angle → servo` conversion against known-good values. No hardware needed.
- **Live** (`--live` flag): sends commands to the physical robot via `/tmp/esp32-proxy.socket`.

## Prerequisites (live mode)

```bash
# On the robot — esp32-proxy must be running
systemctl status mmal_service   # or however it is started on your robot
ls /tmp/esp32-proxy.socket       # must exist
```

## Running the C++ test

```bash
# Build (no ROS 2 needed)
cd ~/ros2_ws/src/mini_pupper_ros/mini_pupper_hardware/test
g++ -std=c++17 -o standalone_hardware_test standalone_hardware_test.cpp -lm

# Math only
./standalone_hardware_test

# Live hardware
./standalone_hardware_test --live
```

## Running the Python test

```bash
cd ~/ros2_ws/src/mini_pupper_ros/mini_pupper_hardware/test

# Math only
python3 standalone_hardware_test.py

# Live hardware
python3 standalone_hardware_test.py --live
```

## Live test sequence

Both tests perform the same sequence on the physical robot:

1. **Read** current servo positions (raw counts 0–1023)
2. **Standing pose** — commands all 12 servos to the calibrated standing position and reads back actual positions. Pass criterion: all 12 within ±5 counts.
3. **Individual servo sweep** — moves each channel in turn from 400 → 512 → 624 with 1 s waits, then returns to neutral. Watch the robot to verify the correct joint/leg moves.

## Expected standing pose servo values

```
RF: abd=498  hip=563  knee=491
LF: abd=526  hip=461  knee=533
RB: abd=526  hip=563  knee=491
LB: abd=498  hip=461  knee=533
```

These values correspond to:

```
abd  = ±0.080 rad
hip  =  1.078 rad
knee_abs = hip + knee = 1.078 + (−1.983) = −0.905 rad
```
