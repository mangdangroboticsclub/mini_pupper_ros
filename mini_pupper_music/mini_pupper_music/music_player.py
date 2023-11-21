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


class MusicPlayer:
    def __init__(self):
        self.audio = pyaudio.PyAudio()
        self.lock = threading.Lock()
        self.playing = False

    def play_music(self, file_path, start_second, duration):
        with self.lock:
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

    def stop_music(self):
        self.playing = False

    def destroy(self):
        self.stop_music()
        self.audio.terminate()
