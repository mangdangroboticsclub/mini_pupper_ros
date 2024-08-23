#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 MangDang
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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from simple_pid import PID
from mini_pupper_interfaces.msg import LineDetectionResult


class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node')
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.interval = 0.25

        self.pid_linear = PID(0.2, 0.01, 0.02, setpoint=0)
        self.pid_linear.output_limits = (-0.01, 0.01)
        self.pid_angular = PID(0.1, 0.01, 0.02, setpoint=0)
        self.pid_angular.output_limits = (-0.2, 0.2)

        self.vel_sub = self.create_subscription(
            LineDetectionResult,
            'velocity',
            self._vel_callback,
            10
        )

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def _vel_callback(self, msg):
        velocity_cmd = Twist()

        if msg.linear != '':
            self.linear_vel = float(msg.linear) * 2  # Scale the linear velocity
            self.angular_vel = float(msg.angular) / 3  # Scale the angular velocity

            control_linear = self.pid_linear(self.linear_vel)
            control_angular = self.pid_angular(self.angular_vel)

            velocity_cmd.linear.y = self.linear_vel + control_linear
            velocity_cmd.angular.z = self.angular_vel + control_angular
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)

        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0.10 / ((abs(float(msg.linear)) + abs(float(msg.angular))) * 3)
        self.vel_publisher_.publish(velocity_cmd)
        time.sleep(self.interval)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
