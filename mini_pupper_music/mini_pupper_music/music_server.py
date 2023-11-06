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
from std_srvs.srv import SetBool
import sounddevice as sd
import soundfile as sf
import threading
import os
from ament_index_python.packages import get_package_share_directory
import ctypes


class SoundPlayerNode(Node):
    def __init__(self):
        super().__init__('mini_pupper_music_service')
        self.service = self.create_service(
            SetBool,
            'music_command',
            self.play_sound_callback
        )
        self.is_playing = False
        self.playback_thread = None
        self.lock = threading.Lock()
        self.load_sound_data()

    def load_sound_data(self):
        package_name = 'mini_pupper_music'
        file_name = 'resource/robot1.wav'
        package_path = get_package_share_directory(package_name)
        sound_file = os.path.join(package_path, file_name)
        try:
            self.sound_data, self.sound_fs = sf.read(sound_file, dtype='float32')
        except Exception as e:
            self.get_logger().error('Failed to load sound data: {}'.format(str(e)))

    def play_sound_callback(self, request, response):
        if request.data:
            with self.lock:
                if not self.is_playing:
                    self.play_sound()
                    response.success = True
                    response.message = 'Sound playback started.'
                else:
                    response.success = False
                    response.message = 'Sound is already playing.'
        else:
            with self.lock:
                if self.is_playing:
                    self.stop_sound()
                    response.success = True
                    response.message = 'Sound playback stopped.'
                else:
                    response.success = False
                    response.message = 'No sound is currently playing.'
        return response

    def play_sound(self):
        self.is_playing = True
        self.playback_thread = threading.Thread(target=self.play_sound_thread)
        self.playback_thread.start()

    def play_sound_thread(self):
        while self.is_playing:
            self.get_logger().info('Playing the song from the beginning')
            sd.play(self.sound_data, self.sound_fs)
            sd.wait()

    def stop_sound(self):
        self.is_playing = False
        if self.playback_thread is not None:
            self.playback_thread.join(timeout=1.0)  # Wait for 1 second for the thread to finish
            if self.playback_thread.is_alive():
                # If the thread is still running, terminate it forcefully
                self.get_logger().warning('Playback thread did not terminate gracefully.')
                self.get_logger().warning('Terminating forcefully.')
                self.terminate_thread(self.playback_thread)

    def terminate_thread(self, thread):
        if not thread.is_alive():
            return

        thread_id = thread.ident
        # Terminate the thread using ctypes
        ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(thread_id),
            ctypes.py_object(SystemExit)
        )
        self.get_logger().warning('Playback thread terminated forcefully.')


def main(args=None):
    rclpy.init(args=args)
    sound_player_node = SoundPlayerNode()
    rclpy.spin(sound_player_node)
    sound_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
