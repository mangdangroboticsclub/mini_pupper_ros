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


from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_simulation')

    default_world = PathJoinSubstitution([this_package, 'worlds', 'mini_pupper_home.world'])

    world = LaunchConfiguration('world')
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo world path'
    )

    gz_sim_launch_path = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={
            'gz_args': [world, ' -r'],
        }.items()
    )

    return LaunchDescription([
        world_launch_arg,
        gz_sim_launch
    ])
