#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022 MangDang
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

# This program is based on https://github.com/champ/champ.
# which is released under the BSD-3-Clause License.
# https://spdx.org/licenses/BSD-3-Clause.html
#
# Copyright (c) 2019-2020 Juan Miguel Jimeno
#
# https://github.com/chvmp/champ/blob/f76d066d8964c8286afbcd9d5d2c08d781e85f54/champ_gazebo/launch/gazebo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_package = get_package_share_directory('mini_pupper_gazebo')
    bringup_package = get_package_share_directory('mini_pupper_bringup')

    default_world_path = os.path.join(
        gazebo_package, 'worlds', 'playground.world')

    bringup_launch_path = os.path.join(
        bringup_package, 'launch', 'bringup.launch.py')

    robot_name = LaunchConfiguration("robot_name")
    sim = LaunchConfiguration("sim")
    joint_hardware_connected = LaunchConfiguration("joint_hardware_connected")
    rviz = LaunchConfiguration("rviz")
    lite = LaunchConfiguration("lite")
    world = LaunchConfiguration("world"),
    world_init_x = LaunchConfiguration("world_init_x"),
    world_init_y = LaunchConfiguration("world_init_y"),
    world_init_z = LaunchConfiguration("world_init_z"),

    world_init_heading = LaunchConfiguration("world_init_heading"),
    gui = LaunchConfiguration("gui"),

    declare_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="false",
        description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        name="robot_name",
        default_value="mini_pupper",
        description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        name="lite",
        default_value="false",
        description="Lite"
    )

    declare_world = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        description="Gazebo world path"
    )
    declare_gui = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument(
        name="world_init_x",
        default_value="0.0"
    )
    declare_world_init_y = DeclareLaunchArgument(
        name="world_init_y",
        default_value="0.0"
    )
    declare_world_init_z = DeclareLaunchArgument(
        name="world_init_z",
        default_value="0.1"
    )
    declare_world_init_heading = DeclareLaunchArgument(
        name="world_init_heading",
        default_value="0.6"
    )
    declare_sim = DeclareLaunchArgument(
        name='sim',
        default_value='true',
        description='Enable use_sime_time to true'
    )
    declare_joint_hardware_connected = DeclareLaunchArgument(
        name='joint_hardware_connected',
        default_value='false',
        description='Set to true if connected to a physical robot'
    )

    mini_pupper_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            "use_sim_time": sim,
            "robot_name": robot_name,
            "gazebo": sim,
            "rviz": rviz,
            "joint_hardware_connected": joint_hardware_connected,
            "publish_foot_contacts": "true",
            "close_loop_odom": "true"
        }.items(),
    )
    champ_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("champ_gazebo"), "launch", "gazebo.launch.py")),
        launch_arguments={
            "use_sim_time": sim,
            "robot_name": robot_name,
            "world": world,
            "lite": lite,
            "world_init_x": world_init_x,
            "world_init_y": world_init_y,
            "world_init_z": world_init_z,
            "world_init_heading": world_init_heading,
            "gui": gui,
            "close_loop_odom": "true",
        }.items(),
    )

    return LaunchDescription([
        declare_rviz,
        declare_robot_name,
        declare_lite,
        declare_world,
        declare_gui,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_z,
        declare_world_init_heading,
        declare_sim,
        declare_joint_hardware_connected,
        mini_pupper_bringup_launch,
        champ_gazebo_launch
    ])
