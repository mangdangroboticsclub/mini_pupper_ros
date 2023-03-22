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
from sensor_msgs.msg import JointState
from MangDang.mini_pupper.HardwareInterface import HardwareInterface


class ServoInterface(Node):
    def __init__(self):
        super().__init__('servo_interface')
        self.subscriber = self.create_subscription(
            JointState, '/joint_command',
            self.cmd_callback, 1)
        self.hardware_interface = HardwareInterface()

    def cmd_callback(self, msg):
        joint_angles = np.array(msg.position).reshape(3, 4)
        self.hardware_interface.set_actuator_postions(joint_angles)

def main(args=None):
    rclpy.init(args=args)
    servo_interface_node = ServoInterface()
    rclpy.spin(servo_interface_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
