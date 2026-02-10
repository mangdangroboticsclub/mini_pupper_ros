#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022 MangDang
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
# @Author  : Zhengxiao Han

import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from MangDang.mini_pupper.HardwareInterface import HardwareInterface


class ServoInterface(Node):
    def __init__(self):
        super().__init__('servo_interface')
        
        # Log at startup with WARN level so it's always visible
        self.get_logger().warn('========================================')
        self.get_logger().warn('Servo Interface Node Starting...')
        self.get_logger().warn('========================================')
        
        self.subscriber = self.create_subscription(
            JointTrajectory, 'joint_group_effort_controller/joint_trajectory',
            self.cmd_callback, 1)
        self.hardware_interface = HardwareInterface()
        
        # Create timer to periodically read servo positions (1Hz)
        self.read_timer = self.create_timer(1.0, self.read_servo_positions)
        
        self.get_logger().info('Servo Interface Node initialized successfully')
        self.get_logger().info('Listening on: joint_group_effort_controller/joint_trajectory')
        self.get_logger().info('Reading servo positions every 1 second')

    def cmd_callback(self, msg):
        joint_positions = msg.points[0].positions
        lf1_position = joint_positions[0]
        lf2_position = joint_positions[1]
        lf3_position = joint_positions[2]
        rf1_position = joint_positions[3]
        rf2_position = joint_positions[4]
        rf3_position = joint_positions[5]
        lb1_position = joint_positions[6]
        lb2_position = joint_positions[7]
        lb3_position = joint_positions[8]
        rb1_position = joint_positions[9]
        rb2_position = joint_positions[10]
        rb3_position = joint_positions[11]

        # Debug: Log received joint angles from controller (every 100 calls)
        if not hasattr(self, '_cmd_counter'):
            self._cmd_counter = 0
        self._cmd_counter += 1
        
        if self._cmd_counter % 100 == 0:
            self.get_logger().info('Joint angles received from controller (rad):')
            self.get_logger().info(f'  LF [abd={lf1_position:.3f}, hip={lf2_position:.3f}, knee={lf3_position:.3f}]')
            self.get_logger().info(f'  RF [abd={rf1_position:.3f}, hip={rf2_position:.3f}, knee={rf3_position:.3f}]')
            self.get_logger().info(f'  LB [abd={lb1_position:.3f}, hip={lb2_position:.3f}, knee={lb3_position:.3f}]')
            self.get_logger().info(f'  RB [abd={rb1_position:.3f}, hip={rb2_position:.3f}, knee={rb3_position:.3f}]')

        # Calculate absolute knee angles (hip + knee)
        lf_knee_abs = lf2_position + lf3_position
        rf_knee_abs = rf2_position + rf3_position
        lb_knee_abs = lb2_position + lb3_position
        rb_knee_abs = rb2_position + rb3_position

        if self._cmd_counter % 100 == 0:
            self.get_logger().info(f'Absolute knee angles (hip+knee): LF={lf_knee_abs:.3f} RF={rf_knee_abs:.3f} LB={lb_knee_abs:.3f} RB={rb_knee_abs:.3f}')

        joint_angles = np.array([
            [rf1_position, lf1_position, rb1_position, lb1_position],
            [rf2_position, lf2_position, rb2_position, lb2_position],
            [rf_knee_abs, lf_knee_abs, rb_knee_abs, lb_knee_abs]
        ])

        self.hardware_interface.set_actuator_postions(joint_angles)

    def read_servo_positions(self):
        """Periodically read and log servo positions from hardware"""
        # Access ESP32Interface through HardwareInterface -> PWMParams -> esp32
        try:
            positions = self.hardware_interface.pwm_params.esp32.servos_get_position()
            
            if positions is None or len(positions) != 12:
                self.get_logger().warn('Failed to read servo positions from hardware')
                return
            
            self.get_logger().warn('Read servo positions from hardware:')
            self.get_logger().warn(f'  RF: abd={positions[0]}, hip={positions[1]}, knee_abs={positions[2]}')
            self.get_logger().warn(f'  LF: abd={positions[3]}, hip={positions[4]}, knee_abs={positions[5]}')
            self.get_logger().warn(f'  RB: abd={positions[6]}, hip={positions[7]}, knee_abs={positions[8]}')
            self.get_logger().warn(f'  LB: abd={positions[9]}, hip={positions[10]}, knee_abs={positions[11]}')
        except Exception as e:
            self.get_logger().error(f'Error reading servo positions: {e}')


def main(args=None):
    rclpy.init(args=args)
    servo_interface_node = ServoInterface()
    rclpy.spin(servo_interface_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
