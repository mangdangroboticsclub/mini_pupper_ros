#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sounddevice as sd
import soundfile as sf
import threading
import os
from ament_index_python.packages import get_package_share_directory


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
        # Load and play the sound file continuously
        package_name = 'mini_pupper_music'
        file_name = 'resource/robot1.wav'
        package_path = get_package_share_directory(package_name)
        sound_file = os.path.join(package_path, file_name)

        data, fs = sf.read(sound_file, dtype='float32')
        while self.is_playing:
            sd.play(data, fs)
            sd.wait()

    def stop_sound(self):
        self.is_playing = False
        if self.playback_thread is not None:
            self.playback_thread.join(timeout=0)
            if self.playback_thread.is_alive():
                # If the thread is still running, stop the sound playback
                sd.stop()


def main(args=None):
    rclpy.init(args=args)
    sound_player_node = SoundPlayerNode()
    rclpy.spin(sound_player_node)
    sound_player_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
