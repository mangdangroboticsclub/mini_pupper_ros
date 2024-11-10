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


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

ROBOT_MODEL = os.getenv('ROBOT_MODEL', default='mini_pupper_2')


def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_simulation')

    default_world_path = PathJoinSubstitution([this_package, 'worlds', 'mini_pupper_home.world'])
    world = LaunchConfiguration('world')
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Gazebo world path'
    )

    world_init_x = LaunchConfiguration('world_init_x')
    world_init_x_launch_arg = DeclareLaunchArgument(
        name='world_init_x',
        default_value='0.0'
    )

    world_init_y = LaunchConfiguration('world_init_y')
    world_init_y_launch_arg = DeclareLaunchArgument(
        name='world_init_y',
        default_value='0.0'
    )

    world_init_z = LaunchConfiguration('world_init_z')
    world_init_z_launch_arg = DeclareLaunchArgument(
        name='world_init_z',
        default_value='0.066'
    )

    world_init_heading = LaunchConfiguration('world_init_heading')
    world_init_heading_launch_arg = DeclareLaunchArgument(
        name='world_init_heading',
        default_value='0.0'
    )

    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_bringup'), 'launch', 'bringup.launch.py']
    )
    mini_pupper_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'use_sim_time': 'True',
            'hardware_connected': 'False'
        }.items()
    )

    gazebo_launch_path = PathJoinSubstitution([this_package, 'launch', 'gazebo.launch.py'])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'world': world
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', ROBOT_MODEL,
            '-x', world_init_x,
            '-y', world_init_y,
            '-z', world_init_z,
            '-R', '0',
            '-P', '0',
            '-Y', world_init_heading
        ],
        output='screen'
    )

    ros2_controllers_launch_path = PathJoinSubstitution([
        this_package,
        'launch',
        'ros2_controllers.launch.py'
    ])
    ros2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_controllers_launch_path)
    )

    links_map_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_description'), 'config', 'champ', ROBOT_MODEL, 'links.yaml']
    )
    contact_sensor_launch = Node(
        package='champ_gazebo',
        executable='contact_sensor',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            links_map_path  # Load parameters from the YAML file,
        ]
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[ros2_controllers_launch, contact_sensor_launch]
            )
        ),
        world_launch_arg,
        world_init_x_launch_arg,
        world_init_y_launch_arg,
        world_init_z_launch_arg,
        world_init_heading_launch_arg,
        mini_pupper_bringup_launch,
        gazebo_launch,
        spawn_entity
    ])
