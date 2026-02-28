#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 MangDang
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

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class SmoothPositionTest(Node):

    def __init__(self):
        super().__init__('smooth_position_test')

        # Publisher for position commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.get_logger().info('DOG RUNNING GAIT SIMULATION - 30 seconds')
        self.get_logger().info('Publishing to: /joint_group_position_controller/commands')

        # Running gait parameters
        self.time_elapsed = 0.0
        self.gait_frequency = 1.5  # 1.5 Hz - slower running speed (was 2.0)
        self.dt = 0.08  # 12.5Hz update rate (was 0.05 = 20Hz)

        # Send position commands at high frequency for smoothness
        self.timer = self.create_timer(self.dt, self.update_running_gait)

        # Stop after 30 seconds
        self.stop_timer = self.create_timer(30.0, self.stop_running)

    def update_running_gait(self):
        """Update robot with realistic running dog gait pattern."""
        self.time_elapsed += self.dt

        # Calculate gait cycle phase (0 to 1)
        gait_phase = (self.time_elapsed * self.gait_frequency) % 1.0

        # Running gait parameters
        body_bounce = 0.1   # Vertical body movement

        # Trot gait: diagonal legs move together
        # Phase 0-0.5: LF+RB up, RF+LB down
        # Phase 0.5-1.0: RF+LB up, LF+RB down

        positions = [0.0] * 12

        # Calculate leg positions based on gait phase
        for leg in range(4):  # 4 legs: LF=0, RF=1, LB=2, RB=3
            base_idx = leg * 3  # Each leg has 3 joints

            # Determine if this leg is in swing (up) or stance (down) phase
            if leg == 0 or leg == 3:  # Left front (0) and Right back (3) - diagonal pair 1
                leg_phase = gait_phase
            else:  # Right front (1) and Left back (2) - diagonal pair 2
                leg_phase = (gait_phase + 0.5) % 1.0

            # Calculate joint angles for proper dog gait
            if leg_phase < 0.5:  # Swing phase - leg lifted and moving forward
                # Hip: small side movement (abduction/adduction)
                hip_side = 0.1 * math.sin(leg_phase * 2 * math.pi)
                # Upper leg: lift up during swing
                upper_angle = 0.6 + 0.3 * math.sin(leg_phase * 2 * math.pi)
                # Lower leg: bend more when lifted
                lower_angle = -1.2 - 0.4 * math.sin(leg_phase * 2 * math.pi)
            else:  # Stance phase - leg on ground, supporting body
                # Hip: minimal side movement during stance
                hip_side = 0.05 * math.sin((leg_phase - 0.5) * 2 * math.pi)
                # Upper leg: more extended for support
                upper_angle = 0.3 + body_bounce * math.sin(gait_phase * 4 * math.pi)
                # Lower leg: extended to reach ground
                lower_angle = -0.8 - body_bounce * 0.5

            # Apply to joints with correct mapping
            positions[base_idx] = hip_side        # Hip joint (side movement)
            positions[base_idx + 1] = upper_angle  # Upper leg joint (up/down)
            positions[base_idx + 2] = lower_angle  # Lower leg joint (extension)

        # Create and publish message
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)

        # Log progress every 5 seconds
        if int(self.time_elapsed) % 5 == 0 and int(self.time_elapsed * 20) % 100 == 0:
            remaining = 30 - int(self.time_elapsed)
            self.get_logger().info(
                f'Running gait active - {remaining}s remaining (Phase: {gait_phase:.2f})')

    def stop_running(self):
        """Stop the running gait and return to neutral."""
        self.get_logger().info('30 seconds completed - stopping run')

        # Return to neutral position
        neutral_msg = Float64MultiArray()
        neutral_msg.data = [0.0] * 12
        self.publisher.publish(neutral_msg)

        # Shutdown after brief delay
        self.create_timer(2.0, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)

    node = SmoothPositionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
