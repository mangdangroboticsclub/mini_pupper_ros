#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .math_operations import euler_from_quaternion, quaternion_from_euler


class PoseController(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose, 'body_pose', 10)
        timer_period = 0.001  # seconds
        self.publisher_timer = self.create_timer(timer_period,
                                                 self.publisher_timer_callback)

        self.subscription = self.create_subscription(Pose,
                                                     'reference_body_pose',
                                                     self.listener_callback,
                                                     10)
        self.subscription  # prevent unused variable warning

        self.interpolation_timer = self.create_timer(
            timer_period,
            self.interpolation_timer_callback
        )

        self.increment = 0.001  # resolution of interpolation
        self.pose_state = Pose()
        self.pose_reference = Pose()

    def publisher_timer_callback(self):
        self.publisher_.publish(self.pose_state)

    def listener_callback(self, msg):
        self.pose_reference = msg

    def interpolation_timer_callback(self):
        state_r, state_p, state_y = euler_from_quaternion(
            self.pose_state.orientation.x,
            self.pose_state.orientation.y,
            self.pose_state.orientation.z,
            self.pose_state.orientation.w
            )

        reference_r, reference_p, reference_y = euler_from_quaternion(
            self.pose_reference.orientation.x,
            self.pose_reference.orientation.y,
            self.pose_reference.orientation.z,
            self.pose_reference.orientation.w
        )

        if (reference_r < state_r):
            state_r = state_r - self.increment
        elif (reference_r > state_r):
            state_r = state_r + self.increment
        if (reference_p < state_p):
            state_p = state_p - self.increment
        elif (reference_p > state_p):
            state_p = state_p + self.increment
        if (reference_y < state_y):
            state_y = state_y - self.increment
        elif (reference_y > state_y):
            state_y = state_y + self.increment

        x, y, z, w = quaternion_from_euler(state_r, state_p, state_y)
        self.pose_state.orientation.x = x
        self.pose_state.orientation.y = y
        self.pose_state.orientation.z = z
        self.pose_state.orientation.w = w

        self.pose_state.position.x = self.pose_reference.position.x
        self.pose_state.position.y = self.pose_reference.position.y
        self.pose_state.position.z = self.pose_reference.position.z


def main(args=None):
    rclpy.init(args=args)
    minimal_controller = PoseController()
    rclpy.spin(minimal_controller)
    minimal_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()