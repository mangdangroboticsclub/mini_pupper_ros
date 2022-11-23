# Plan for Upgrading to ROS2 Humble

There was an effort upgrading Mini Pupper to ROS2 galactic. Basically, we made the simulation work in Gazebo. However, we paused the work due to many missing features in galactic and it's end of life for the community support already. 

Humble Hawksbill (humble) is the most important ROS2 distribution so far. It is a long-term support (LTS) release and it has become ROS2 community's primary choice for adding new features or maintaining. **We are going to upgrade Mini Pupper's software to humble.**


## Goal

- By end of Feburary 2023, Mini Pupper runs with humble for all the ROS1 features like slam and navigation.
- By end of March 2023, Mini Pupper 2 runs with humble.


## Work To Be Done

### 1. Upgrade hardware drivers to Ubuntu 22.04

1.1. QuadrupedRobot
1.2. mini_pupper_bsp: hardware drivers for servo and display.
1.3. ds4drv: Sony DualShock 4 userspace driver.

**Outcome**: Mini Pupper can walk properly controlled by PS4 controller.


### 2. Upgrade champ projects from galactic to humble

2.1. champ  
2.2. champ_robots
2.3. champ_teleop  

**Outcome**: champ and mini_pupper shall be able to do slam in simulation 


### 3. Upgrade mini_pupper_ros to humble

3.1. cartographer: use [ROS2 version]https://github.com/ros2/cartographer if possible. Otherwise, find alternatives. 

3.2. ldlidar_stl_ros: use [ROS2 version](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2). Might need to request them and work them to support humble.

3.3. mini_pupper_description: update to latest physics/design in urdf to be provided by Afreez.

3.4. mini_pupper_control, mini_pupper_teleop: update launch files and scripts to run in humble and ubuntu 22.04.

3.5. mini_pupper_gazebo, mini_pupper_navigation, mini_pupper_bringup: mainly update launch files. Quite some work has been done during galactic upgrade. 

3.6. mini_pupper_examples: community to support.

**Outcome**: Mini Pupper can do slam in real world.


### 4. Apply humble to Mini Pupper 2

4.1. Setup git repo/branch.
4.2. Update robot description urdf.
4.3. Update hardware drivers.
4.4. Integrate new sensors.
4.5. Testing and Tuning.

**Outcome**: Mini Pupper 2 can do slam in real world.

