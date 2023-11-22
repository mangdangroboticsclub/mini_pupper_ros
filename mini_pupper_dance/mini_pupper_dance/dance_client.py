#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import DanceCommand
from mini_pupper_interfaces.srv import PlayMusic, StopMusic
from .episode import dance_commands
from .episode import dance_song_file_name
from .episode import dance_song_start_second


class MiniPupperDanceClientAsync(Node):

    def __init__(self):
        super().__init__('mini_pupper_dance_client_async')
        self.dance_cli = self.create_client(DanceCommand, 'dance_command')
        self.play_music_cli = self.create_client(PlayMusic, 'play_music')
        self.stop_music_cli = self.create_client(StopMusic, 'Stop_music')

        while not self.dance_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.dance_commands = dance_commands

    def send_dance_request(self, dance_command):
        req = DanceCommand.Request()
        req.data = dance_command
        future = self.dance_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_play_music_request(self, file_name, start_second):
        req = PlayMusic.Request()
        req.file_name = file_name
        req.start_second = start_second
        self.play_music_cli.call_async(req)  # Fire-and-forget style communication

    def send_stop_music_request(self):
        req = StopMusic.Request()
        self.stop_music_cli.call_async(req)  # Fire-and-forget style communication


def main():
    rclpy.init()
    minimal_client = MiniPupperDanceClientAsync()

    for index, command in enumerate(minimal_client.dance_commands):
        # Start music for the first command
        if index == 0:
            minimal_client.get_logger().info('Starting music...')
            minimal_client.send_play_music_request(dance_song_file_name,
                                                   dance_song_start_second)

        # Send movemoment comment for the robot to dance
        response = minimal_client.send_dance_request(command)
        if response.executed:
            minimal_client.get_logger().info('Command Executed!')

        # Stop music after the last command
        if index == len(minimal_client.dance_commands) - 1:
            minimal_client.get_logger().info('Stopping music...')
            minimal_client.send_stop_music_request()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
