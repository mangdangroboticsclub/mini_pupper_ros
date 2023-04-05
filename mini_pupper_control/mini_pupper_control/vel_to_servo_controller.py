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
#
# This program is based on
# https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/master/run_robot.py
# which is released under the MIT License.
# Copyright (c) 2020 Stanford Student Robotics
# https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/62277f64dc4d1b293feddc8ecd0986d144f656d6/LICENSE

import numpy as np
# from .StanfordQuadruped.src.IMU import IMU
from .StanfordQuadruped.src.Controller import Controller
from .StanfordQuadruped.src.Command import Command
from .StanfordQuadruped.src.State import State
from .StanfordQuadruped.pupper.Kinematics import four_legs_inverse_kinematics
# In case to run in simulation
try:
    from MangDang.mini_pupper.Config import Configuration
except ImportError:
    from .StanfordQuadruped.pupper.Config import Configuration

# ROS 2 related packages
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class VirtualDisplay():
    def __init__(self) -> None:
        pass

    def show_state(self, virtual_behavior_state):
        pass


class VelocityToServoController(Node):

    def __init__(self):
        super().__init__("velocity_to_servo_controller")

        # Parameters
        self.receive_topic_name = "/cmd_vel"
        self.FIRST_TIME_FLAG = 1
        self.TIMER_INTERVAL = 0.02
        self.disp = VirtualDisplay()

        # Motion Control
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_linear_z = 0.0
        self.target_angular_x = 0.0
        self.target_angular_y = 0.0
        self.target_angular_z = 0.0

        # Standford Quadruped Initialization
        self.config = Configuration()
        self.controller = Controller(self.config, four_legs_inverse_kinematics)
        self.state = State()
        self.initial_angles = self.state.joint_angles

        # Publisher & Subscriber

        self.publisher = self.create_publisher(JointState, 'joint_command', 50)
        self.subscriber = self.create_subscription(
            Twist, self.receive_topic_name, self.listener_callback, 1)
        self.subscriber  # prevent unused variable warning

        # Timer
        self.timer = self.create_timer(
            self.TIMER_INTERVAL, self.data_processor)

        # Human Behaviors Counter
        self.counter = 0

    def constrain_movement(self, command, config):
        """
        Constrain the movement of the robot.

        Prevent the robot movement from being too fast.
        """
        # constrain horizontal velocity
        command.horizontal_velocity[0] = np.clip(
            command.horizontal_velocity[0],
            -config.max_x_velocity,
            config.max_x_velocity
        )
        command.horizontal_velocity[1] = np.clip(
            command.horizontal_velocity[1],
            -config.max_y_velocity,
            config.max_y_velocity
        )

        # constrain yaw rate
        command.yaw_rate = np.clip(
            command.yaw_rate, -config.max_yaw_rate, config.max_yaw_rate)
        return command

    def publish_joint_positions(self, joint_angles):
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = joint_angles.flatten().tolist()
        self.publisher.publish(joint_state_msg)

    def listener_callback(self, twist_msg):
        self.target_linear_x = twist_msg.linear.x
        self.target_linear_y = twist_msg.linear.y
        self.target_angular_z = twist_msg.angular.z
        self.target_linear_z = twist_msg.linear.z
        self.target_angular_x = twist_msg.angular.x
        self.target_angular_y = twist_msg.angular.y

    def data_processor(self):
        # IMU TODO
        # self.quat_orientation = (
        #         imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
        #     )
        self.quat_orientation = np.array([1, 0, 0, 0])
        self.state.quat_orientation = self.quat_orientation

        # Command Formation
        command = Command()
        command.horizontal_velocity = np.array(
            [self.target_linear_x, self.target_linear_y])
        command.yaw_rate = self.target_angular_z
        command = self.constrain_movement(command, self.config)
        # Simulate Human Behaviors
        if (self.counter == 0):
            self.counter += 1
        elif (self.counter == 1):
            command.activate_event = True
            self.counter += 1
        elif (self.counter == 2):
            self.initial_angles = self.state.joint_angles
            self.counter += 1
        elif (self.counter == 3):
            command.trot_event = True
            self.counter += 1
        else:
            pass

        # If Speed is 0, Stop Trotting.
        STOP = False
        if (self.target_linear_x == 0
            and self.target_linear_y == 0
                and self.target_angular_z == 0):
            STOP = True

        # Main
        if (command.activate_event is not True):
            # State Update
            self.controller.run(self.state, command, self.disp)

            # Hardware Control
            if not STOP:
                self.publish_joint_positions(self.state.joint_angles)
            else:
                self.publish_joint_positions(self.initial_angles)


def print_gait_parameters(config):
    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)


def main(args=None):
    # Create imu handle
    # use_imu = False
    # if use_imu:
    #     imu = IMU(port="/dev/ttyACM0")
    #     imu.flush_buffer()
    rclpy.init(args=args)
    velocity_to_servo_controller = VelocityToServoController()
    rclpy.spin(velocity_to_servo_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
