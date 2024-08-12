#!/usr/bin/env python3

import math

# Conversion between quaternions and Euler angles
# refer to the following link
#    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

# this implementation assumes normalized quaternion
# converts to Euler angles in 3-2-1 sequence


def euler_from_quaternion(x, y, z, w):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (w * z + x * y)
    cosy_cosp = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)

    return roll_x, pitch_y, yaw_z  # in radius


# roll (x), pitch (Y), yaw (z)
def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w