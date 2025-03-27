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
        self.publisher_ = self.create_publisher(Command, 'robot_command', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.is_trotting = False

    def cmd_vel_callback(self, msg):
        command = self.create_command(msg)
        self.publisher_.publish(command)
        self.get_logger().info(f'Published Command: horizontal_velocity=({command.horizontal_velocity[0]}, {command.horizontal_velocity[1]}), yaw_rate={command.yaw_rate}')
        self.get_logger().info(f'Published Command: trot_event=({command.trot_event}, self.is_trotting={self.is_trotting}')

    def create_command(self, cmd_vel):
        command = Command()
        command.height = -0.07
        is_cmd_zero = np.allclose([cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z], 0, atol=1e-3)
        command.trot_event = (self.is_trotting and is_cmd_zero ) or (not self.is_trotting and not is_cmd_zero)
        self.is_trotting = not is_cmd_zero
        
        # default standing locations
        matrix = Matrix3x4()
        matrix.row1 = [0.06, 0.06, -0.06, -0.06]
        matrix.row2 = [-0.05, 0.05, -0.05, 0.05]
        matrix.row3 = [-0.07, -0.07, -0.07, -0.07]
        command.foot_location = matrix

        x_vel = min(self.config.max_x_velocity, cmd_vel.linear.x)
        y_vel = min(self.config.max_y_velocity, cmd_vel.linear.y)
        yaw_rate = min(self.config.max_yaw_rate, cmd_vel.angular.z)
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = yaw_rate
        return command

def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = TwistToCommandNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()