#!/usr/bin/env python3
#
# Copyright 2023 MangDang
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

from MangDang.mini_pupper.ESP32Interface import ESP32Interface

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


GRAVITY = 9.80665
DEG2RAD = math.pi / 180.0


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_interface')
        self.get_logger().info("Initializing IMU interface")

        self.freq = self.declare_parameter('freq', 100).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value

        self.get_logger().info("Creating IMU hardware interface")
        self.esp32_interface = ESP32Interface()
        self.initialized = False
        self.gyro_offset = [0, 0, 0]
        self.acc_offset = [0, 0, 0]
        self.calibration_count = 200

        self.get_logger().info("Creating IMU publisher")
        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0 / self.freq, self.timer_callback)

    def read_imu(self):
        raw_data = self.esp32_interface.imu_get_data()

        # Differnet direction of x-axis and y-axis
        return [
            raw_data['ay'] * GRAVITY,
            raw_data['ax'] * GRAVITY,
            raw_data['az'] * -GRAVITY,
            raw_data['gy'] * DEG2RAD,
            raw_data['gx'] * DEG2RAD,
            raw_data['gz'] * -DEG2RAD,
        ]

    def timer_callback(self):
        msg = Imu()

        ax, ay, az, gx, gy, gz = self.read_imu()

        if not self.initialized:
            self.acc_offset[0] += ax
            self.acc_offset[1] += ay
            self.acc_offset[2] += az
            self.gyro_offset[0] += gx
            self.gyro_offset[1] += gy
            self.gyro_offset[2] += gz

            self.calibration_count -= 1
            if self.calibration_count == 0:
                self.acc_offset[0] /= 200
                self.acc_offset[1] /= 200
                self.acc_offset[2] /= 200
                self.acc_offset[2] -= GRAVITY
                self.gyro_offset[0] /= 200
                self.gyro_offset[1] /= 200
                self.gyro_offset[2] /= 200

                self.initialized = True
                self.get_logger().info("IMU calibration finished")
            else:
                return

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = ax - self.acc_offset[0]
        msg.linear_acceleration.y = ay - self.acc_offset[1]
        msg.linear_acceleration.z = az - self.acc_offset[2]
        msg.angular_velocity.x = gx - self.gyro_offset[0]
        msg.angular_velocity.y = gy - self.gyro_offset[1]
        msg.angular_velocity.z = gz - self.gyro_offset[2]
        msg.orientation_covariance[0] = -1
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    rclpy.shutdown()
    node.esp32_interface.close()


if __name__ == '__main__':
    main()
