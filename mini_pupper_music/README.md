## Quick Guide

You can call service in terminal as below:

```
# To play the music
ros2 service call /play_music mini_pupper_interfaces/srv/PlayMusic "{file_name: 'robot1.mp3', start_second: 3}"
```

```
# To stop the music
ros2 service call /stop_music mini_pupper_interfaces/srv/StopMusic
```






