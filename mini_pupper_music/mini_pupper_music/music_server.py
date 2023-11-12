#!/usr/bin/env python3
#
# Copyright 2023 MangDang
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
# @Author  : Cullen SUN

import rclpy
from rclpy.node import Node
from mini_pupper_interfaces.srv import MusicCommand
import pygame
import concurrent.futures
import os
from ament_index_python.packages import get_package_share_directory


class SoundPlayerNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_music_service')
        self.service = self.create_service(
            MusicCommand,
            'music_command',
            self.play_sound_callback
        )
        self.song_pool = {'robot1.mp3', 'robot1.wav'}
        self.thread_executor = concurrent.futures.ThreadPoolExecutor(max_workers=1) 

    def play_sound_callback(self, request, response):
        if request.command == 'play':
            if request.file_name in self.song_pool:
                self.play_sound_file(request.file_name)
                response.success = True
                response.message = 'Sound playback started.'
            else:
                response.success = False
                response.message = f'File {request.file_name} is not found.'

        else:
            response.success = False
            response.message = f'Command {request.command} is not supported.'

        return response

    def play_sound_file(self, file_name):
        package_name = 'mini_pupper_music'
        package_path = get_package_share_directory(package_name)
        sound_path = os.path.join(package_path, 'resource', file_name)

        # Submit the playback task to the thread pool
        self.thread_executor.submit(self.play_sound_in_background, sound_path)

    def play_sound_in_background(self, sound_path):
        pygame.init()
        pygame.mixer.music.load(sound_path)
        pygame.mixer.music.play()


def main(args=None):
    rclpy.init(args=args)
    sound_player_node = SoundPlayerNode()
    rclpy.spin(sound_player_node)
    sound_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
