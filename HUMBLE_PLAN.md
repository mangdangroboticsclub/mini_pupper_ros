# Plan for Upgrading to ROS 2 Humble

There was an effort upgrading Mini Pupper to ROS 2 galactic. Basically, we made the simulation work in Gazebo. However, we paused the work due to many missing features in galactic and it's end of life for the official support at the end of November 2022. 

Humble Hawksbill (humble) is the most important ROS 2 distribution so far. It is a long-term support (LTS) release and it has become ROS 2 community's primary choice for adding new features or maintaining. **We are going to upgrade Mini Pupper's software to humble.**


## Goal

1. By end of Feburary 2023, Mini Pupper runs with humble for all the ROS1 features like slam and navigation.
2. In summer of 2023, Mini Pupper 2 runs with humble.


## Work To Be Done

### 1. Upgrade hardware drivers to Ubuntu 22.04

1. [mangdangroboticsclub/StanfordQuadruped](https://github.com/mangdangroboticsclub/StanfordQuadruped): fork of the Stanford Pupper with modification to make it run on Mini Pupper
2. [mangdangroboticsclub/mini_pupper_bsp](https://github.com/mangdangroboticsclub/mini_pupper_bsp): hardware drivers for servo and display.
3. [ds4drv](https://github.com/chrippa/ds4drv): Sony DualShock 4 userspace driver.

Note: Thanks to [@hdumcke](https://github.com/hdumcke), these steps have been almost done in September 2022.

**Outcome**: Mini Pupper can walk properly controlled by PS4 controller.


### 2. Upgrade champ projects from galactic to humble

1. champ  
2. champ_robots
3. champ_teleop  
4. mini_pupper_description: update to latest physics/design in urdf to be provided by MangDang.

Note: If we found some error caused by Gazebo (formerly called Ignition), we may postpone this goal.

**Outcome**: champ and mini_pupper shall be able to do slam in simulation 


### 3. Upgrade mini_pupper_ros to humble

1. cartographer: use ROS 2 version ([ros2/cartographer_ros](https://github.com/ros2/cartographer_ros), [ros2/cartographer](https://github.com/ros2/cartographer)) if possible. Otherwise, find alternatives. 

2. ldlidar_stl_ros: use ROS 2 version ([ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)). Might need to request them and work them to support humble or try the community maintained version ([Myzhar/ldrobot-lidar-ros2](https://github.com/Myzhar/ldrobot-lidar-ros2)).

3. mini_pupper_driver, mini_pupper_teleop: update launch files and scripts to run in humble and ubuntu 22.04.

4. mini_pupper_gazebo, mini_pupper_navigation, mini_pupper_bringup: mainly update launch files. Quite some work has been done during galactic upgrade. 

5. mini_pupper_examples: community to support.

**Outcome**: Mini Pupper can do slam in real world.


### 4. Apply humble to Mini Pupper 2

1. Setup git repo/branch.
2. Update robot description urdf.
3. Update hardware drivers.
4. Integrate new sensors, like IMU.
5. Testing and Tuning.

**Outcome**: Mini Pupper 2 can do slam in real world.
