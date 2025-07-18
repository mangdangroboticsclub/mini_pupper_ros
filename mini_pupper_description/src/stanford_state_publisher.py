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
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class StanfordStatePublisher(Node):
    def __init__(self):
        super().__init__('stanford_state_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_filtered_madgwick',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.rotation = msg.orientation
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = StanfordStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
