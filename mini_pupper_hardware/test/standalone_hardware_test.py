#!/usr/bin/env python3
"""
Standalone Python hardware test - mirrors standalone_hardware_test.cpp.
Mirrors servo_interface.py usage of HardwareInterface.

Run on the robot (no ROS2 required):
  python3 standalone_hardware_test.py          # math-only test (no hardware)
  python3 standalone_hardware_test.py --live   # send to real ESP32 socket

NOTE: ESP32Interface.servos_set_position() uses torque=1 (broken!).
      HardwareInterface.set_actuator_postions() uses torque=500 (correct).
      This test exercises BOTH paths so you can compare.
"""

import math
import sys
import time

import numpy as np

# ============================================================
# Mirror of mini_pupper_hardware.hpp calibration constants
# ============================================================
NEUTRAL_POSITION = 512.0
MICROS_PER_RAD = (760.0 - 210.0) / math.pi
NEUTRAL_ANGLES_RAD = [0.0, math.pi / 4.0, -math.pi / 4.0]

# [axis][leg]:  leg 0=RF, 1=LF, 2=RB, 3=LB
SERVO_MULTIPLIERS = [
    [ 1,  1, -1, -1],   # axis 0 (abduction)
    [-1,  1, -1,  1],   # axis 1 (hip)
    [-1,  1, -1,  1],   # axis 2 (knee)
]

LEG_NAMES  = ["RF", "LF", "RB", "LB"]
AXIS_NAMES = ["abd", "hip", "knee"]

SERVO_LABELS = [
    "RF-abd", "RF-hip", "RF-knee",
    "LF-abd", "LF-hip", "LF-knee",
    "RB-abd", "RB-hip", "RB-knee",
    "LB-abd", "LB-hip", "LB-knee",
]

# Standing pose angles (matches mini_pupper_2_controllers.yaml)
LF_ABD, RF_ABD, LB_ABD, RB_ABD = -0.080, 0.080, -0.080, 0.080
HIP_ANGLE  =  1.078
KNEE_ANGLE = -1.983
KNEE_ABS   = HIP_ANGLE + KNEE_ANGLE   # -0.905


def angle_to_servo_position(angle_rad, axis_index, leg_index):
    """Mirrors C++ angle_to_servo_position()"""
    neutral = NEUTRAL_ANGLES_RAD[axis_index]
    mult    = SERVO_MULTIPLIERS[axis_index][leg_index]
    deviation = (angle_rad - neutral) * mult
    pos = NEUTRAL_POSITION - MICROS_PER_RAD * deviation
    if math.isnan(pos):
        return 0
    return int(round(max(0.0, min(1023.0, pos))))


def build_standing_pose():
    return [
        angle_to_servo_position(RF_ABD,   0, 0),   # [0]  RF-abd
        angle_to_servo_position(HIP_ANGLE, 1, 0),  # [1]  RF-hip
        angle_to_servo_position(KNEE_ABS,  2, 0),  # [2]  RF-knee
        angle_to_servo_position(LF_ABD,   0, 1),   # [3]  LF-abd
        angle_to_servo_position(HIP_ANGLE, 1, 1),  # [4]  LF-hip
        angle_to_servo_position(KNEE_ABS,  2, 1),  # [5]  LF-knee
        angle_to_servo_position(RB_ABD,   0, 2),   # [6]  RB-abd
        angle_to_servo_position(HIP_ANGLE, 1, 2),  # [7]  RB-hip
        angle_to_servo_position(KNEE_ABS,  2, 2),  # [8]  RB-knee
        angle_to_servo_position(LB_ABD,   0, 3),   # [9]  LB-abd
        angle_to_servo_position(HIP_ANGLE, 1, 3),  # [10] LB-hip
        angle_to_servo_position(KNEE_ABS,  2, 3),  # [11] LB-knee
    ]


def print_servo_array(label, positions):
    print(f"{label}: {positions}")
    print(f"  RF[abd={positions[0]} hip={positions[1]} knee={positions[2]}]"
          f"  LF[abd={positions[3]} hip={positions[4]} knee={positions[5]}]")
    print(f"  RB[abd={positions[6]} hip={positions[7]} knee={positions[8]}]"
          f"  LB[abd={positions[9]} hip={positions[10]} knee={positions[11]}]")


# ============================================================
# MATH TEST
# ============================================================
def run_math_test():
    print("\n========== MATH TEST ==========")
    print("Verifying angle_to_servo_position() against Python baseline\n")

    tests = [
        ("RF abd +0.080",       0.080,         0, 0, 498),
        ("RF hip  1.078",       1.078,         1, 0, 563),
        ("RF knee_abs -0.905",  -0.905,        2, 0, 491),
        ("LF abd -0.080",       -0.080,        0, 1, 526),
        ("LF hip  1.078",       1.078,         1, 1, 461),
        ("LF knee_abs -0.905",  -0.905,        2, 1, 533),
        ("RB abd +0.080",       0.080,         0, 2, 526),
        ("LB abd -0.080",       -0.080,        0, 3, 498),
        ("RF abd neutral",      0.0,           0, 0, 512),
        ("LF abd neutral",      0.0,           0, 1, 512),
        ("RF hip neutral",      math.pi / 4.0, 1, 0, 512),
        ("LF hip neutral",      math.pi / 4.0, 1, 1, 512),
    ]

    passed = failed = 0
    for name, angle, axis, leg, expected in tests:
        got = angle_to_servo_position(angle, axis, leg)
        ok  = (got == expected)
        extra = "" if ok else f" (expected {expected})"
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: angle={angle} -> servo={got}{extra}")
        if ok: passed += 1
        else:  failed += 1

    print(f"\nResult: {passed} passed, {failed} failed")

    standing = build_standing_pose()
    print("\n--- Standing pose servo positions ---")
    print_servo_array("Standing pose", standing)
    print("\nExpected from Python:")
    print("  RF[abd=498 hip=563 knee=491]  LF[abd=526 hip=461 knee=533]")
    print("  RB[abd=526 hip=563 knee=491]  LB[abd=498 hip=461 knee=533]")

    return standing


# ============================================================
# LIVE TEST
# ============================================================
def compare_positions(commanded, actual, label=""):
    if label:
        print(f"\n[{label}] Comparing commanded vs actual:")
    any_error = False
    for i in range(12):
        diff = actual[i] - commanded[i]
        ok   = abs(diff) <= 5
        if not ok:
            any_error = True
        mark   = "    " if ok else "!!! "
        status = "OK"          if ok else "<-- MISMATCH"
        print(f"  {mark}{SERVO_LABELS[i]:<10}: commanded={commanded[i]:4d}  actual={actual[i]:4d}  diff={diff:+4d}  {status}")
    if any_error:
        print("\n*** MISMATCHES DETECTED ***")
    else:
        print("\nAll servos reached commanded positions within tolerance!")
    return not any_error


def run_live_test(standing):
    print("\n========== LIVE HARDWARE TEST (Python) ==========")

    try:
        from MangDang.mini_pupper.ESP32Interface import ESP32Interface
        from MangDang.mini_pupper.HardwareInterface import HardwareInterface
    except ImportError as e:
        print(f"Cannot import MangDang modules: {e}")
        print("Is the MangDang Python module installed? (pip install or BSP setup)")
        return

    esp32 = ESP32Interface()
    torque_on  = [1]   * 12  # Binary enable (BB12B12H: torque is uint8, 1=enabled)
    torque_1   = [1]   * 12  # Same - kept for comparison test

    # ── [1] Read current positions ────────────────────────────────────────────
    print("\n[1] Reading current servo positions...")
    before = esp32.servos_get_position()
    if before:
        print_servo_array("  Current", before)

    # ── [2a] Send via servos_set_position_torque with torque=1 (correct protocol) ──
    print("\n[2a] Sending standing pose via servos_set_position_torque(torque=1)...")
    print("     (BB12B12H protocol: torque is uint8 binary enable, 1=enabled)")
    print_servo_array("  Commanding", standing)
    esp32.servos_set_position_torque(standing, torque_on)
    print("  Waiting 2s...")
    time.sleep(2)

    after_500 = esp32.servos_get_position()
    if after_500:
        print_servo_array("  Actual  ", after_500)
        compare_positions(standing, after_500, "2a torque=500")

    # ── [2b] Send via servos_set_position (uses internal torque=1) ────────────
    print("\n[2b] Returning to neutral, then sending via servos_set_position()...")
    print("     (Uses ESP32Interface.servos_set_position internally, torque=1)")
    neutral = [512] * 12
    esp32.servos_set_position_torque(neutral, torque_on)
    time.sleep(1)

    esp32.servos_set_position(standing)   # uses torque=1 internally
    print("  Waiting 2s...")
    time.sleep(2)

    after_1 = esp32.servos_get_position()
    if after_1:
        print_servo_array("  Actual  ", after_1)
        compare_positions(standing, after_1, "2b torque=1 (broken)")

    # ── [2c] Send via HardwareInterface (the actual servo_interface.py path) ─
    print("\n[2c] Returning to neutral, then sending via HardwareInterface...")
    print("     (This is the exact path used by servo_interface.py)")
    esp32.servos_set_position_torque(neutral, torque_on)
    time.sleep(1)

    joint_angles = np.array([
        [RF_ABD,   LF_ABD,   RB_ABD,   LB_ABD],
        [HIP_ANGLE, HIP_ANGLE, HIP_ANGLE, HIP_ANGLE],
        [KNEE_ABS,  KNEE_ABS,  KNEE_ABS,  KNEE_ABS],
    ])
    hw = HardwareInterface()
    hw.set_actuator_postions(joint_angles)
    print("  Waiting 2s...")
    time.sleep(2)

    after_hw = esp32.servos_get_position()
    if after_hw:
        print_servo_array("  Actual  ", after_hw)
        compare_positions(standing, after_hw, "2c HardwareInterface")

    # ── [3] Individual servo sweep ────────────────────────────────────────────
    print("\n[3] Individual servo sweep (torque=1 binary enable, each channel 400→624)...")
    print("    Watch the PHYSICAL robot - which leg/joint moves?")

    esp32.servos_set_position_torque(neutral, torque_on)
    time.sleep(1)

    for i in range(12):
        print(f"  Channel {i:2d} ({SERVO_LABELS[i]})... ", end="", flush=True)

        cmd = list(neutral)
        cmd[i] = 400
        esp32.servos_set_position_torque(cmd, torque_on)
        time.sleep(1)
        pos = esp32.servos_get_position()
        actual_400 = pos[i] if pos else -1

        cmd[i] = 624
        esp32.servos_set_position_torque(cmd, torque_on)
        time.sleep(1)
        pos2 = esp32.servos_get_position()
        actual_624 = pos2[i] if pos2 else -1

        rng  = abs(actual_624 - actual_400)
        flag = "  <-- SERVO NOT RESPONDING!" if rng < 50 else ""
        print(f"400->actual={actual_400}, 624->actual={actual_624}{flag}")

        # Return to neutral
        cmd[i] = 512
        esp32.servos_set_position_torque(cmd, torque_on)
        time.sleep(0.3)

    esp32.close()
    print("\nDone.")


# ============================================================
# main
# ============================================================
def main():
    live = "--live" in sys.argv

    print("Mini Pupper Hardware Standalone Test (Python)")
    print("===============================================")

    standing = run_math_test()

    if live:
        run_live_test(standing)
    else:
        print("\n[Live test skipped - run with --live to test actual hardware]")
        print("Example: python3 standalone_hardware_test.py --live")


if __name__ == "__main__":
    main()
