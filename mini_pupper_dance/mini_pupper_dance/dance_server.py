#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import DanceCommand
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import time
from .math_operations import quaternion_from_euler


class MiniPupperDanceService(Node):

    def __init__(self):
        super().__init__('mini_pupper_dance_service')
        self.srv = self.create_service(DanceCommand,
                                       'dance_command',
                                       self._dance_callback)

        self.vel_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_publisher_ = self.create_publisher(Pose,
                                                     'reference_body_pose', 10)
        self.interval = 0.5  # seconds

    def _dance_callback(self, request, response):
        velocity_cmd = Twist()
        pose_cmd = Pose()
        if (request.data == 'move_forward'):
            velocity_cmd.linear.x = 0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'move_backward'):
            velocity_cmd.linear.x = -0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'move_left'):
            velocity_cmd.linear.y = 0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'move_right'):
            velocity_cmd.linear.y = -0.5
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'turn_left'):
            velocity_cmd.angular.z = 1.0
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'turn_right'):
            velocity_cmd.angular.z = -1.0
            self.vel_publisher_.publish(velocity_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'look_up'):
            pose_cmd.orientation.x,
            pose_cmd.orientation.y,
            pose_cmd.orientation.z,
            pose_cmd.orientation.w = quaternion_from_euler(0.0, -0.3, 0.0)

            self.pose_publisher_.publish(pose_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'look_down'):
            pose_cmd.orientation.x,
            pose_cmd.orientation.y,
            pose_cmd.orientation.z,
            pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.3, 0.0)

            self.pose_publisher_.publish(pose_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'look_left'):
            pose_cmd.orientation.x,
            pose_cmd.orientation.y,
            pose_cmd.orientation.z,
            pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, 0.3)

            self.pose_publisher_.publish(pose_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'look_right'):
            pose_cmd.orientation.x,
            pose_cmd.orientation.y,
            pose_cmd.orientation.z,
            pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, -0.3)

            self.pose_publisher_.publish(pose_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'look_middle'):
            pose_cmd.orientation.x,
            pose_cmd.orientation.y,
            pose_cmd.orientation.z,
            pose_cmd.orientation.w = quaternion_from_euler(0.0, 0.0, 0.0)

            self.pose_publisher_.publish(pose_cmd)
            self.get_logger().info('Publishing: "%s"' % request.data)
            time.sleep(self.interval)  # Make sure the robot moved

        elif (request.data == 'stay'):
            time.sleep(self.interval)  # do nothing

        else:
            self.get_logger().info('Invalid command: "%s"' % request.data)
            time.sleep(self.interval)  # do nothing

        # Stop the robot from moving
        velocity_cmd = Twist()
        self.vel_publisher_.publish(velocity_cmd)

        # Give response
        response.executed = True
        return response


def main():
    rclpy.init()
    minimal_service = MiniPupperDanceService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
