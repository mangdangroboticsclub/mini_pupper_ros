## Introduction 

This package provides a music audio file playback service. The main interface is described in the "mini_pupper_interfaces" package. For more details, please refer to the "PlayMusic.srv" and "StopMusic.srv" files. This package is used by "mini_pupper_dance" to play a song during a dance.

Please note that the audio files should be placed in the "audio" folder of the package, alongside files like "robot1.mp3" and "robot1.wav". If you add a new file, you will need to rebuild the package for it to work properly.

## Quick Guide

1. First of all, please start the music service in the Mini Pupper.

```
ros2 launch mini_pupper_music music.launch.py
```

2. You can call the service in another terminal using the following commands:

```
# To play the music
ros2 service call /play_music mini_pupper_interfaces/srv/PlayMusic "{file_name: 'robot1.mp3', start_second: 3}"
```

```
# To stop the music
ros2 service call /stop_music mini_pupper_interfaces/srv/StopMusic
```
