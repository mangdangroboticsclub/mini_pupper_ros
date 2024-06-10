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


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ROBOT_MODEL = os.getenv('ROBOT_MODEL', default="mini_pupper_2")


def generate_launch_description():
    if ROBOT_MODEL == "mini_pupper_2":
        description_package = FindPackageShare('mini_pupper_2_description')
    else:
        description_package = FindPackageShare('mini_pupper_description')

    links_map_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', 'links.yaml']
    )

    this_package = FindPackageShare('mini_pupper_gazebo')
    default_world_path = PathJoinSubstitution([this_package, 'worlds', 'mini_pupper_home.world'])

    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_bringup'), 'launch', 'bringup.launch.py']
    )

    champ_gazebo_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_gazebo'), 'launch', 'gazebo.launch.py']
    )

    world = LaunchConfiguration("world")
    world_launch_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        description="Gazebo world path"
    )

    world_init_x = LaunchConfiguration("world_init_x")
    world_init_x_launch_arg = DeclareLaunchArgument(
        name="world_init_x",
        default_value="0.0"
    )

    world_init_y = LaunchConfiguration("world_init_y")
    world_init_y_launch_arg = DeclareLaunchArgument(
        name="world_init_y",
        default_value="0.0"
    )

    world_init_z = LaunchConfiguration("world_init_z")
    world_init_z_launch_arg = DeclareLaunchArgument(
        name="world_init_z",
        default_value="0.066"
    )

    world_init_heading = LaunchConfiguration("world_init_heading")
    world_init_heading_launch_arg = DeclareLaunchArgument(
        name="world_init_heading",
        default_value="0.0"
    )

    mini_pupper_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={"simulation": "True"}.items(),
    )
    champ_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(champ_gazebo_launch_path),
        launch_arguments={
            "robot_name": ROBOT_MODEL,
            "world": world,
            "links_map_path": links_map_path,
            "world_init_x": world_init_x,
            "world_init_y": world_init_y,
            "world_init_z": world_init_z,
            "world_init_heading": world_init_heading
        }.items(),
    )

    return LaunchDescription([
        world_launch_arg,
        world_init_x_launch_arg,
        world_init_y_launch_arg,
        world_init_z_launch_arg,
        world_init_heading_launch_arg,
        mini_pupper_bringup_launch,
        champ_gazebo_launch
    ])
