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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')
        self.get_logger().info("Webcam Publisher Node Created")
        
        # publisher to image_raw
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        
        # bridge for conversion between opencv and ros image messages
        self.bridge = CvBridge()
        
        # init webcam
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam!")
            return
            
        # Set camera properties for better performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Timer to publish frames at just above 30 FPS
        self.timer = self.create_timer(0.033, self.publish_frame)
        
        self.get_logger().info("Webcam publisher started, publishing to /image_raw")

    def publish_frame(self):
        """Capture frame from webcam and publish as ROS Image message"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam")
            return
            
        try:
            # convert from opencv to ros image message (bgr8 is default opencv encoding)
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # set header
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'

            self.publisher.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Webcam publisher shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()