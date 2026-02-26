#
# Copyright 2025 MangDang (www.mangdang.net)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import numpy as np


def deadband(value, band_radius):
    return max(value - band_radius, 0) + min(value + band_radius, 0)


def clipped_first_order_filter(input, target, max_rate, tau):
    rate = (target - input) / tau
    return np.clip(rate, -max_rate, max_rate)


def convert_to_command_positions(joint_angles):
    # Convert stanford controller joint angles to JointTrajectoryPoint
    # positions
    rf1_position = joint_angles[0, 0]
    lf1_position = joint_angles[0, 1]
    rb1_position = joint_angles[0, 2]
    lb1_position = joint_angles[0, 3]

    rf2_position = joint_angles[1, 0]
    lf2_position = joint_angles[1, 1]
    rb2_position = joint_angles[1, 2]
    lb2_position = joint_angles[1, 3]

    rf3_position = joint_angles[2, 0]
    lf3_position = joint_angles[2, 1]
    rb3_position = joint_angles[2, 2]
    lb3_position = joint_angles[2, 3]

    joint_positions = [
        lf1_position, lf2_position, lf3_position,
        rf1_position, rf2_position, rf3_position,
        lb1_position, lb2_position, lb3_position,
        rb1_position, rb2_position, rb3_position
    ]

    return joint_positions
