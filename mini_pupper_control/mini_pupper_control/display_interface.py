#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 MangDang
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# @Author  : Yunlong Feng

import PIL
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
        self.sub = self.create_subscription(Image, 'mini_pupper_lcd/image_raw')
        self.get_logger().info("Creating LCD hardware interface")
        self.disp = ST7789()
        self.disp.begin()
        self.disp.clear()

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
