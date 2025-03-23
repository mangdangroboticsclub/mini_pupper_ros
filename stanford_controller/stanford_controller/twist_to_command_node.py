import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mini_pupper_interfaces.msg import Command
import numpy as np

class TwistToCommandNode(Node):
    def __init__(self):
        super().__init__('twist_to_command_node')
        self.publisher_ = self.create_publisher(Command, 'robot_command', 10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        command_msg = Command()
        command_msg.horizontal_velocity = np.array([msg.linear.x, msg.linear.y])
        command_msg.yaw_rate = msg.angular.z
        self.publisher_.publish(command_msg)
        self.get_logger().info(f'Published Command: horizontal_velocity=({command_msg.horizontal_velocity[0]}, {command_msg.horizontal_velocity[1]}), yaw_rate={command_msg.yaw_rate}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistToCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()