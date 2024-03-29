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

import threading
import pyaudio
from pydub import AudioSegment
from pydub.utils import make_chunks
import sounddevice


class MusicPlayer:
    def __init__(self):
        # sounddevice is not used in any logics, just print something here
        # See: https://stackoverflow.com/a/76305661/663645
        print(sounddevice.query_devices())

        self.audio = pyaudio.PyAudio()
        self.playing = False
        self.play_thread = None

    def _play_music(self, file_path, start_second, duration):
        file_extension = file_path.split(".")[-1]
        if duration == 0.0:
            duration = None

        self.playing = True

        audio_seg = AudioSegment.from_file(
            file=file_path,
            format=file_extension,
            start_second=start_second,
            duration=duration
        )

        stream = self.audio.open(
            format=self.audio.get_format_from_width(audio_seg.sample_width),
            channels=audio_seg.channels,
            rate=audio_seg.frame_rate,
            output=True
        )

        try:
            for chunk in make_chunks(audio_seg, 500):
                if self.playing:
                    stream.write(chunk._data)
                else:
                    break
        finally:
            stream.stop_stream()
            stream.close()
            self.playing = False

    def start_music(self, file_path, start_second=0, duration=0.0):
        self.play_thread = threading.Thread(
            target=self._play_music,
            args=(file_path, start_second, duration),
            daemon=True
        )
        self.play_thread.start()

    def stop_music(self):
        self.playing = False

    def destroy(self):
        self.stop_music()
        self.audio.terminate()
        self.play_thread.join()
