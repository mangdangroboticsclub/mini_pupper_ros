## Quick Start
We recently migrated Stanford controller to the `mini_pupper_ros` repo, enabling enhanced control and new features for Mini Pupper. This migration allows for smoother and more dynamic dance routines.

### How to make Mini Pupper dance

#### Single robot dancing

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
ros2 launch mini_pupper_dance dance.launch.py 
```

#### Multi robot dancing

**Mini Pupper**
```sh
# Terminal 1 (ssh)
# Robot 1 SSH Terminal
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py multi_robot:=true robot_namespace:=robot1

# Robot 2 SSH Terminal
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py multi_robot:=true robot_namespace:=robot2

# Robot 3 SSH Terminal
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_bringup bringup.launch.py multi_robot:=true robot_namespace:=robot3
```

**PC (Or Mini Pupper)**
```sh
source ~/ros2_ws/install/setup.bash
ros2 launch mini_pupper_dance multi_robot_dance.launch.py multi_robot:=true robot_count:=3
```



### How to modify
You can modify the file createDanceActionListSample.py in the mini_pupper_dance folder to define new dance moves by editing or adding actions to the dance action list. Each action specifies a pose or movement for the robot, and modifying this file allows you to create custom dance routines. After making changes, rebuild the package to apply the updates.
