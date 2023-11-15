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
import threading
from pydub import AudioSegment
from pydub.playback import play
import os
from ament_index_python.packages import get_package_share_directory


class MusicServiceNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_music_service')
        self.service = self.create_service(
            MusicCommand,
            'music_command',
            self.play_music_callback
        )
        self.song_pool = {'robot1.mp3', 'robot1.wav'}
        self.playing_lock = threading.Lock()

    def play_music_callback(self, request, response):
        if request.command == 'play':
            if request.file_name in self.song_pool:
                if not self.playing_lock.locked():
                    self.play_sound_file(request.file_name,
                                         request.start_second,
                                         request.duration)
                    response.success = True
                    response.message = 'Sound playback started.'
                else:
                    response.success = False
                    response.message = 'Another sound is already playing.'
            else:
                response.success = False
                response.message = f'File {request.file_name} is not found.'

        else:
            response.success = False
            response.message = f'Command {request.command} is not supported.'

        return response

    def play_sound_file(self, file_name, start_second, duration):
        # Create a new thread for playing the sound
        thread = threading.Thread(
            target=self.play_sound_in_background,
            args=(file_name, start_second, duration)
        )
        # Set the thread as a daemon (will exit when the main program ends)
        thread.daemon = True
        thread.start()

    def play_sound_in_background(self, file_name, start_second, duration):
        with self.playing_lock:
            package_name = 'mini_pupper_music'
            package_path = get_package_share_directory(package_name)
            sound_path = os.path.join(package_path, 'resource', file_name)
            file_extension = file_name.split(".")[-1]
            self.get_logger().info(f'Play music at {sound_path}')
            self.get_logger().info(f'File format {file_extension}')

            audio = AudioSegment.from_file(
                file=sound_path,
                format=file_extension,
                start_second=start_second,
                duration=duration
            )

            play(audio)


def main(args=None):
    rclpy.init(args=args)
    music_service_node = MusicServiceNode()
    rclpy.spin(music_service_node)
    music_service_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
