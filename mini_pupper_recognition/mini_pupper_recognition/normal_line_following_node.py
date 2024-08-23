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
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node')
        self.angular = 0.0
        self.interval = 0.5
        self.speed = 0.0

        self.linear_sub = self.create_subscription(
            String,
            'linear_vel',
            self._linear_callback,
            10
        )

        self.angular_sub = self.create_subscription(
            String,
            'angular_vel',
            self._angular_callback,
            10
        )

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def _angular_callback(self, msg):
        self.angular = float(msg.data)

    def _linear_callback(self, linear):

        velocity_cmd = Twist()

        if linear.data != '':
            velocity_cmd.linear.y = float(linear.data) / 10
            velocity_cmd.angular.z = self.angular
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)

        velocity_cmd = Twist()
        velocity_cmd.linear.x = 0.10 / ((abs(float(linear.data)) + abs(self.angular)) * 3)
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
