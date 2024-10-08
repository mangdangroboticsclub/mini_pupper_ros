#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022-2023 MangDang
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
# @Author  : Yunlong Feng

import PIL
import PIL.Image
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from MangDang.LCD.ST7789 import ST7789


class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_interface')
        self.get_logger().info("Initializing display interface")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, 'mini_pupper_lcd/image_raw', self.callback, 10)
        self.get_logger().info("Creating LCD hardware interface")
        self.disp = ST7789()
        self.disp.begin()

    def callback(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        image = PIL.Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        resized = image.resize((320, 240))
        self.disp.display(resized)


def main(args=None):
    rclpy.init(args=args)

    display_node = DisplayNode()

    rclpy.spin(display_node)

    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
