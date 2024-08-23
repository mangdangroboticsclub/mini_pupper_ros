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
from mini_pupper_interfaces.msg import AiLineRecognitionResult


class CloudLineFollowingNode(Node):
    def __init__(self):
        super().__init__('cloud_line_following_node')
        self.interval = 2.0
        self.speed = 0.0
        self.previous_direction = 'center'
        self.walk_previous_direction = 0
        self.previous_orientation = ''

        self.image_sub = self.create_subscription(
            AiLineRecognitionResult,
            'image',
            self._image_callback,
            10
        )

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def _image_callback(self, msg):

        velocity_cmd = Twist()

        if msg.direction == 'left' or msg.direction == 'right':
            self.previous_direction = msg.direction

        if msg.direction == 'left' or msg.direction == 'right':
            if msg.orientation == 'vertical' or msg.orientation == 'slanted':
                self.previous_orientation = msg.orientation

        if msg.direction == 'left':
            if msg.orientation == 'vertical':
                self.speed = 0.04
                velocity_cmd.linear.y = 0.025 * float(msg.extent)
                velocity_cmd.angular.z = 0.0
            elif msg.orientation == 'slanted':
                self.speed = 0.03
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = 0.4 * float(msg.extent)
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)
            self.walk_previous_direction = 0

        elif msg.direction == 'right':
            if msg.orientation == 'vertical':
                self.speed = 0.04
                velocity_cmd.linear.y = -0.025 * float(msg.extent)
                velocity_cmd.angular.z = 0.0
            elif msg.orientation == 'slanted':
                self.speed = 0.03
                velocity_cmd.linear.y = 0.0
                velocity_cmd.angular.z = -0.4 * float(msg.extent)
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(self.interval)
            self.walk_previous_direction = 0

        elif msg.direction == 'center':
            self.speed = 0.04
            self.vel_publisher_.publish(velocity_cmd)
            time.sleep(1)
            self.walk_previous_direction = 0

        elif msg.direction == '' and self.walk_previous_direction < 4:

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

        elif msg.direction == '' and self.walk_previous_direction >= 4:

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
    node = CloudLineFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
