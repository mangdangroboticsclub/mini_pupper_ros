#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022-2023 MangDang
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    description_package = FindPackageShare('mini_pupper_description')

    description_path = PathJoinSubstitution(
        [description_package, 'urdf', 'mini_pupper_description.urdf.xacro']
    )

    rviz_config_path = PathJoinSubstitution(
        [description_package, 'rviz', 'urdf_viewer.rviz']
    )
    servo_interface_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_control'), 'launch', 'servo_interface.launch.py']
    )

    vel_to_servo_control_interface_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_driver'), 'launch', 'vel_to_servo_control_interface.launch.py']
    )
    lidar_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_bringup'), 'launch', 'lidar.launch.py']
    )

    robot_name = LaunchConfiguration("robot_name")
    sim = LaunchConfiguration("sim")
    joint_hardware_connected = LaunchConfiguration("joint_hardware_connected")
    rviz = LaunchConfiguration("rviz")

    declare_robot_name = DeclareLaunchArgument(
            name='robot_name',
            default_value='mini_pupper',
            description='Set robot name for multi robot'
        )

    declare_sim = DeclareLaunchArgument(
            name='sim',
            default_value='false',
            description='Enable use_sime_time to true'
        )

    declare_rviz = DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        )

    declare_hardware_connected = DeclareLaunchArgument(
            name='joint_hardware_connected',
            default_value='true',
            description='Set to true if connected to a physical robot'
        )

    rviz2_node = Node(
            package="rviz2",
            namespace="",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            condition=IfCondition(rviz)
        )

    servo_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(servo_interface_launch_path),
        condition=IfCondition(joint_hardware_connected),
    )

    vel_to_servo_control_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vel_to_servo_control_interface_launch_path),
        condition=IfCondition(joint_hardware_connected),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(joint_hardware_connected),
    )

    return LaunchDescription([
        declare_robot_name,
        declare_sim,
        declare_rviz,
        declare_hardware_connected,
        rviz2_node,
        vel_to_servo_control_interface_launch,
        servo_interface_launch,
        lidar_launch,
    ])
