## Quick Start
We recently migrated Stanford controller to the `mini_pupper_ros` repo, enabling enhanced control and new features for Mini Pupper. This migration allows for smoother and more dynamic dance routines.

### How to make Mini Pupper dance

**Mini Pupper**
```sh
# Terminal 1 (ssh)
. ~/ros2_ws/install/setup.bash # setup.zsh if you use zsh instead of bash
ros2 launch mini_pupper_bringup bringup.launch.py launch_twist_converter:=false
```

**PC (Or Mini Pupper)**
```sh
# Terminal 2 (ssh)
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_dance new_dance.launch.py
```

### 3.2 How to modify
You can modify the file createDanceActionListSample.py in the new_dance folder to define new dance moves by editing or adding actions to the dance action list. Each action specifies a pose or movement for the robot, and modifying this file allows you to create custom dance routines. After making changes, rebuild the package to apply the updates.
