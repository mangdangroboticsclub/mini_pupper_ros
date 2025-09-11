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


class CurvatureCompensationNode(Node):
    def __init__(self):
        super().__init__("curvature_compensation")

        self.declare_parameter("drift_correction", 0.0)
        self.drift_correction = self.get_parameter("drift_correction").value

        # Subscribe to nav input
        self.sub = self.create_subscription(
            Twist, "/cmd_vel_raw", self.compensate_curvature, 10
        )

        # Publish to standard cmd_vel
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info(
            f"Drift compensation active: correction_factor={self.drift_correction}"
        )

    def compensate_curvature(self, msg):
        """Apply drift correction when moving forward"""
        output = Twist()
        output.linear = msg.linear
        output.angular = msg.angular

        if msg.linear.x > 0:
            output.angular.z += self.drift_correction * msg.linear.x

        self.pub.publish(output)


def main():
    rclpy.init()
    node = CurvatureCompensationNode()
    rclpy.spin(node)
