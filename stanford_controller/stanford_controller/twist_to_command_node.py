import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from mini_pupper_interfaces.msg import Command
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

    def cmd_vel_callback(self, msg):
        command_msg = Command()
        x_vel = min(self.config.max_x_velocity, msg.linear.x)
        y_vel = min(self.config.max_y_velocity, msg.linear.y)
        yaw_rate = min(self.config.max_yaw_rate, msg.angular.z)
        command_msg.horizontal_velocity = np.array([x_vel, y_vel])
        command_msg.yaw_rate = yaw_rate
        self.publisher_.publish(command_msg)
        self.get_logger().info(f'Published Command: horizontal_velocity=({command_msg.horizontal_velocity[0]}, {command_msg.horizontal_velocity[1]}), yaw_rate={command_msg.yaw_rate}')

def main(args=None):
    rclpy.init(args=args)
    config = Configuration()
    node = TwistToCommandNode(config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()