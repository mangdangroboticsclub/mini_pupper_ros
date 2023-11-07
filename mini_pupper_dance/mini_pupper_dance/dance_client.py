#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import DanceCommand
from std_srvs.srv import SetBool
from .episode import dance_commands


class MiniPupperDanceClientAsync(Node):

    def __init__(self):
        super().__init__('mini_pupper_dance_client_async')
        self.dance_cli = self.create_client(DanceCommand, 'dance_command')
        self.music_cli = self.create_client(SetBool, 'music_command')
        while not self.dance_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.dance_commands = dance_commands

    def send_dance_request(self, dance_command):
        req = DanceCommand.Request()
        req.data = dance_command
        future = self.dance_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_music_request(self, trigger):
        req = SetBool.Request()
        req.data = trigger
        self.music_cli.call_async(req)  # Fire-and-forget style communication


def main():
    rclpy.init()
    minimal_client = MiniPupperDanceClientAsync()

    for index, command in enumerate(minimal_client.dance_commands):
        # Start music for the first command
        if index == 0:
            minimal_client.get_logger().info('Starting music...')
            minimal_client.send_music_request(True)

        # Send movemoment comment for the robot to dance
        response = minimal_client.send_dance_request(command)
        if response.executed:
            minimal_client.get_logger().info('Command Executed!')

        # Stop music after the last command
        if index == len(minimal_client.dance_commands) - 1:
            minimal_client.get_logger().info('Stopping music...')
            minimal_client.send_music_request(False)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
