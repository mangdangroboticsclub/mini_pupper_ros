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
        self.extent = None
        self.orientation = None
        self.interval = 2.0
        self.speed = 0.0
        self.previous_direction = 'center'
        self.walk_previous_direction = 0
        self.previous_orientation = ''

        self.direction_sub = self.create_subscription(
            String,
            'direction',
            self._direction_callback,
            10
        )

        self.extent_sub = self.create_subscription(
            String,
            'extent_of_movement',
            self._extent_callback,
            10
        )

        self.orientation_sub = self.create_subscription(
            String,
            'orientation_of_movement',
            self._orientation_callback,
            10
        )

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def _extent_callback(self, msg):
        self.extent = msg.data

    def _orientation_callback(self, msg):
        self.orientation = msg.data

    def _direction_callback(self, direction):

        velocity_cmd = Twist()

        if direction.data == 'left' or direction.data == 'right':
            self.previous_direction = direction.data

        if direction.data == 'left' or direction.data == 'right':
            if self.orientation == 'vertical' or self.orientation == 'slanted':
                self.previous_orientation = self.orientation

        if direction.data == 'left':
            if self.orientation == 'vertical':
                self.speed = 0.04
                velocity_cmd.linear.y = 0.025 * float(self.extent)
                velocity_cmd.angular.z = 0.0
            elif self.orientation == 'slanted':
                self.speed = 0.03
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = 0.4 * float(self.extent)
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)
            self.walk_previous_direction = 0

        elif direction.data == 'right':
            if self.orientation == 'vertical':
                self.speed = 0.04
                velocity_cmd.linear.y = -0.025 * float(self.extent)
                velocity_cmd.angular.z = 0.0
            elif self.orientation == 'slanted':
                self.speed = 0.03
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = -0.4 * float(self.extent)
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)
            self.walk_previous_direction = 0

        elif direction.data == 'center':
            self.speed = 0.04
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(1)
            self.walk_previous_direction = 0

        elif direction.data == '' and self.walk_previous_direction < 4:

            self.walk_previous_direction += 1

            self.speed = 0.0
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)

            if self.previous_direction == 'left':
                self.speed = 0.02
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = 0.4
                self.vel_publisher_.publish(velocity_cmd)
                time.sleep(self.interval)

            elif self.previous_direction == 'right':
                self.speed = 0.02
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = -0.4
                self.vel_publisher_.publish(velocity_cmd)
                time.sleep(self.interval)

        elif direction.data == '' and self.walk_previous_direction >= 4:

            velocity_cmd.angular.z = 0.0
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)

            if self.previous_direction == 'left':
                self.speed = 0.02
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = -0.5
                self.vel_publisher_.publish(velocity_cmd)
                time.sleep(self.interval)

            elif self.previous_direction == 'right':
                self.speed = 0.02
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = 0.5
                self.vel_publisher_.publish(velocity_cmd)
                time.sleep(self.interval)

        velocity_cmd = Twist()
        velocity_cmd.linear.x = self.speed
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
