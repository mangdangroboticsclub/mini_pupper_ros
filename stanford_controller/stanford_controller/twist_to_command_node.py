#
# Copyright 2025 MangDang (www.mangdang.net)
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

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from mini_pupper_interfaces.msg import Command
from mini_pupper_interfaces.msg import Matrix3x4
from MangDang.mini_pupper.Config import Configuration


class TwistToCommandNode(Node):

    def __init__(self, config):
        super().__init__('twist_to_command_node')
        self.config = config

        # remember last incoming cmd_vel & when it arrived
        self.last_twist = Twist()
        self.last_twist_time = self.get_clock().now()

        # used to detect rising/falling edges on “non-zero” cmd_vel
        self.prev_zero = True

        # publish at your controller rate
        self.timer = self.create_timer(self.config.dt, self.timer_callback)

        # ROS pubs/subs
        self.publisher_ = self.create_publisher(Command, 'robot_command', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist):
        # simply remember the last twist and its timestamp
        self.last_twist = msg
        self.last_twist_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_twist_time).nanoseconds * 1e-9

        # if we’ve seen a fresh non-zero cmd_vel recently use it, else zero
        use_vel = (elapsed < (self.config.dt * 4) and not self._vel_zero(self.last_twist))
        twist = self.last_twist if use_vel else Twist()

        cmd = self.create_command(twist)
        self.publisher_.publish(cmd)
        self.get_logger().debug(f'Publishing Command | trot_event={cmd.trot_event}')

    def create_command(self, twist: Twist) -> Command:
        cmd = Command()
        cmd.height = -0.07

        # default standing locations
        matrix = Matrix3x4()
        matrix.row1 = [0.06, 0.06, -0.06, -0.06]
        matrix.row2 = [-0.05, 0.05, -0.05, 0.05]
        matrix.row3 = [-0.07, -0.07, -0.07, -0.07]
        cmd.legs_location = matrix

        # clamp both forward and backward
        x_vel = float(np.clip(
            twist.linear.x,
            -self.config.max_x_velocity,
            self.config.max_x_velocity,
        ))
        y_vel = float(np.clip(
            twist.linear.y,
            -self.config.max_y_velocity,
            self.config.max_y_velocity,
        ))
        yaw_rate = float(np.clip(
            twist.angular.z,
            -self.config.max_yaw_rate,
            self.config.max_yaw_rate,
        ))
        cmd.horizontal_velocity = np.array([x_vel, y_vel])
        cmd.yaw_rate = yaw_rate
        cmd.roll = 0.0
        cmd.pitch = 0.0
        cmd.yaw = 0.0

        # detect zero↔non-zero edge and fire trot_event only once
        is_zero = self._vel_zero(twist)
        cmd.trot_event = (self.prev_zero != is_zero)
        self.prev_zero = is_zero

        return cmd

    def _vel_zero(self, twist: Twist) -> bool:
        return np.allclose(
            [twist.linear.x, twist.linear.y, twist.angular.z],
            0.0,
            atol=1e-3
        )


def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = TwistToCommandNode(config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user, shutting down...")
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
