# Mini Pupper ROS2

## 1. Installation
Ubuntu 22.04 + ROS2 Humble is required.
```sh
cd ~
mkdir colcon_ws
cd colcon_ws
mksir src
cd src
git clone https://github.com/mangdangroboticsclub/mini_pupper_ros.git -b ros2
git clone --recursive https://github.com/chvmp/champ.git -b ros2
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
pip install setuptools==58.2.0

# execute on Mini Pupper
colcon build --packages-ignore champ_gazebo mini_pupper_gazebo
sudo apt-get install ros-humble-ros2-controllers
# execute on PC
colcon build
sudo apt-get install ros-humble-ros2-controllers ros-humble-teleop-twist-keyboard ros-humble-cartographer-ros

echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42"
```

## 2. RVIZ Test
```sh
# Terminal 1
# if execute on Mini Pupper
ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=true
# if execute on PC
ros2 launch mini_pupper_bringup bringup.launch.py

# Terminal 2, execute on PC
rviz2 -d src/champ/champ_description/rviz/urdf_viewer.rviz

# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```

## 3. Test in Gazebo
```sh
# Terminal 1, execute on PC
ros2 launch mini_pupper_gazebo gazebo.launch.py

# Terminal 2, execuute on PC
rviz2 -d src/champ/champ_description/rviz/urdf_viewer.rviz

# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```

## 4. Cartographer Test in Gazebo
```sh
# Terminal 1, execute on PC
ros2 launch mini_pupper_gazebo gazebo.launch.py

# Terminal 2, execute on PC
ros2 launch mini_pupper_navigation slam.launch.py use_sim_time:=true

# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```

If you want to save map:
```sh
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '~/mymap.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ~/mymap
```

## 5. Cartographer Test in reality
```sh
# Terminal 1, execute on Mini Pupper
ros2 launch mini_pupper_bringup bringup.launch.py hardware_connected:=true

# Terminal 2, execute on PC
ros2 launch mini_pupper_navigation slam.launch.py

# Terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Then control robot dog with your keyboard
```

If you want to save map:
```sh
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '~/mymap.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ~/mymap
```
