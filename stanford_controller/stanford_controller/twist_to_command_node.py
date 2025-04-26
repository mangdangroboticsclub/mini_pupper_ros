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
        self.last_twist = self.get_clock().now()
        self.timer = self.create_timer(self.config.dt, self.timer_callback)
        self.publisher_ = self.create_publisher(Command, 'robot_command', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.last_command = None
        self.is_trotting = False

    def timer_callback(self):
        elapsed = (self.get_clock().now() - self.last_twist).to_sec()
        if self.last_command is not None and elapsed > self.config.dt * 4:
            # publish the last command
            self.publisher_.publish(self.last_command )
            self.get_logger().debug('Published last command')
        else:
            # if no fresh /cmd_vel, publish default stand‐still command
            command = self.get_default_command()
            self.publisher_.publish(command)
            self.get_logger().debug('Published stand command')

    def cmd_vel_callback(self, msg):
        self.last_twist = self.get_clock().now()
        command = self.create_command(msg)
        self.get_logger().info(
            f'Published Command: \
              horizontal_velocity=({command.horizontal_velocity}, \
              yaw_rate={command.yaw_rate}')
        self.get_logger().info(
            f'Published Command: trot_event=({command.trot_event}, \
              self.is_trotting={self.is_trotting}')
        self.last_command = command

    def get_default_command(self):
        cmd = Command()
        cmd.height = -0.07
        cmd.horizontal_velocity = [0.0, 0.0]
        matrix = Matrix3x4()
        matrix.row1 = [0.06, 0.06, -0.06, -0.06]
        matrix.row2 = [-0.05, 0.05, -0.05, 0.05]
        matrix.row3 = [-0.07, -0.07, -0.07, -0.07]
        cmd.legs_location = matrix
        cmd.yaw_rate = 0.0
        cmd.roll = 0.0
        cmd.pitch = 0.0
        cmd.yaw = 0.0
        cmd.trot_event = self.is_trotting
        self.is_trotting = False
        return cmd

    def create_command(self, cmd_vel):
        cmd = Command()
        cmd.height = -0.07
        is_cmd_zero = np.allclose(
            [cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z],
            0,
            atol=1e-3
        )
        cmd.trot_event = (
            self.is_trotting and is_cmd_zero) or (
            not self.is_trotting and not is_cmd_zero)
        self.is_trotting = not is_cmd_zero

        # default standing locations
        matrix = Matrix3x4()
        matrix.row1 = [0.06, 0.06, -0.06, -0.06]
        matrix.row2 = [-0.05, 0.05, -0.05, 0.05]
        matrix.row3 = [-0.07, -0.07, -0.07, -0.07]
        cmd.legs_location = matrix

        x_vel = min(self.config.max_x_velocity, cmd_vel.linear.x)
        y_vel = min(self.config.max_y_velocity, cmd_vel.linear.y)
        yaw_rate = min(self.config.max_yaw_rate, cmd_vel.angular.z)
        cmd.horizontal_velocity = np.array([x_vel, y_vel])
        cmd.yaw_rate = yaw_rate
        cmd.roll = 0.0
        cmd.pitch = 0.0
        cmd.yaw = 0.0
        return cmd


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
