#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2024 MangDang
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

"""
ROS 2 Jazzy Gazebo Sim (Harmonic) Launch File
Replaces gazebo_ros with ros_gz_sim and includes ros_gz_bridge for topic mapping
"""

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_simulation')

    # World file path (using .sdf for Gazebo Sim)
    default_world = PathJoinSubstitution([this_package, 'worlds', 'mini_pupper_home.sdf'])

    world = LaunchConfiguration('world')
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo Sim world path (.sdf format)'
    )

    # Gazebo Sim launch (replaces gazebo_ros)
    gz_sim_launch_path = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world],
            'on_exit_shutdown': 'True'
        }.items()
    )

    # ROS-Gazebo Bridge (CRITICAL: maps topics between ROS 2 and Gazebo Sim)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Velocity command
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # Odometry
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # Laser scan
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # TF
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            # Camera
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Depth camera
            '/camera/depth@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        world_launch_arg,
        gazebo_launch,
        bridge_node
    ])
