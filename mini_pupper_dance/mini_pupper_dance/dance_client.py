#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import DanceCommand
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import sys
from .episode import * # change different espisodes here

class MiniPupperDanceClientAsync(Node):

    def __init__(self):
        super().__init__('mini_pupper_dance_client_async')
        self.cli = self.create_client(DanceCommand, 'dance_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DanceCommand.Request()

        self.dance_commands = dance_commands

    def send_request(self, dance_command):
        self.req.data = dance_command
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    minimal_client = MiniPupperDanceClientAsync()
    for command in minimal_client.dance_commands:
        response = minimal_client.send_request(command)
        if(response.executed == True):
            minimal_client.get_logger().info('Command Executed!')
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()