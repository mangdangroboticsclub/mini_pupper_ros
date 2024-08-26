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

import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
from mini_pupper_interfaces.msg import LineDetectionResult


def detect_black_line(frame):
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Use adaptive thresholding to isolate the black line
    _, thresh = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

    # Perform morphological operations to clean the image
    kernel = np.ones((3, 3), np.uint8)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Process the largest contour which is most likely the black line
    if contours:
        # Sort contours by area to get the largest one
        largest_contour = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(largest_contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        # Calculate the center of the line
        center_x = int(rect[0][0])

        # Calculate the angle of the line
        angle = rect[2]

        angular = 0.0
        linear = 0.0

        # Calculate direction based on position relative to frame center
        frame_center = frame.shape[1] // 2
        deviation = (center_x - frame_center) / frame_center

        # Calculate extent for slanted lines
        if 0 < angle < 45:
            angular = - angle / 22.5
        elif 45 < angle < 90:
            angular = (90 - angle) / 22.5

        linear = -(deviation)

        return linear, angular

    return '', ''


class LineDetectionNode(Node):
    def __init__(self):
        super().__init__('line_detection_node')
        self.sub = self.create_subscription(Image, '/image_raw', self.line_recognition, 10)
        self.vel_publisher_ = self.create_publisher(LineDetectionResult, 'velocity', 10)

    def line_recognition(self, msg):

        # Convert the Image message to a OpenCV image
        self.bridge = CvBridge()
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect the black line's orientation, extent, and direction
        linear, angular = detect_black_line(cv_img)

        message = LineDetectionResult()
        message.linear = str(linear)
        message.angular = str(angular)
        self.get_logger().info(f"Linear: {linear}\nAngular: {angular}")
        self.vel_publisher_.publish(message)


def main():
    rclpy.init()
    minimal_service = LineDetectionNode()
    rclpy.spin(minimal_service)


if __name__ == '__main__':
    main()
