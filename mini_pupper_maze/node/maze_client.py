#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import MazeCommand


class MiniPupperMazeClientAsync(Node):

    def __init__(self):
        super().__init__('mini_pupper_maze_client_async')
        self.maze_cli = self.create_client(MazeCommand, 'maze_command')

        while not self.maze_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.maze_commands = maze_commands

    def send_maze_request(self, maze_command):
        req = MazeCommand.Request()
        req.data = maze_command
        future = self.maze_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    minimal_client = MiniPupperMazeClientAsync()

    response = minimal_client.send_maze_request(command)



if __name__ == '__main__':
    main()