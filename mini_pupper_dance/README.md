
## Introduction 
The dance function was developed during one of the AWS re:Invent events in 2022. You can find the original code in the [repository](https://github.com/mangdangroboticsclub/mini-pupper-aws) for more details.

In this main repository, we have recreated the "mini_pupper_dance" and "mini_pupper_music" packages based on the original code. The new code is written in Python and includes enhancements in the music service.

The main interface is described in the "mini_pupper_interfaces" package. For more details, please refer to the "DanceCommand.srv", "PlayMusic.srv", and "StopMusic.srv" files.

## 1. Install extra packages for playing music

```sh
sudo apt-get install ffmpeg portaudio19-dev -y
pip install pydub pyaudio
```

## 2. Quick Start

### 2.1 Mini Pupper

```sh
# Terminal 1 (ssh)
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py
```

```sh
# Terminal 2 (ssh)
. ~/ros2_ws/install/setup.bash 
ros2 launch mini_pupper_music music.launch.py
```

### 2.2 PC (Or Mini Pupper)
```sh
# Terminal 3 (ssh)
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_dance dance.launch.py
```

### 3 How to modify

### mini_pupper_music
If you want to add a new audio file, please place it in the "resource" folder of the package, alongside files like "robot1.mp3" and "robot1.wav".

### mini_pupper_dance
This package includes the following Python scripts in the mini_pupper_dance/mini_pupper_dance folder:
- dance_client.py: The client reads and sends dance commands. It sends a service command to play music on the first dance command and sends another service command to stop music on the last dance command.
- dance_server.py: The server receives dance commands and executes them. You can add more dance functions in dance_server.py.
- pose_controller.py: A pose controller for Mini Pupper. You don't need to modify this.
- episode.py: The dancing episode. You should edit your dancing episode here. You can also modify the music file name here.

### Rebuild
If you make any changes, you will need to rebuild the modified packages. After that, you can follow the "Quick Start" instructions again to make the robot dance.