#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
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


class NavVelScaler(Node):
    def __init__(self):
        super().__init__("nav_vel_scaler")
        self.sub = self.create_subscription(Twist, "/cmd_vel_navigation2", self.scale_vel, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
    
    def scale_vel(self, msg):
        scaled = Twist()
        scaled.linear.x = msg.linear.x * 1.7 # 0.0294 -> 0.05, 0.0588 -> 0.10
        scaled.linear.y = msg.linear.y * 2.0
        scaled.angular.z = msg.angular.z * 2.0 # 0.25 -> 0.5, 0.50 -> 1.0
        self.pub.publish(scaled)


def main():
    rclpy.init()
    rclpy.spin(NavVelScaler())