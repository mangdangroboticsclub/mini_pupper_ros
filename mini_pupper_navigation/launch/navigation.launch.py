#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2023 MangDang
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
# from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_navigation')

    default_map_path = PathJoinSubstitution([this_package, 'maps', 'map.yaml'])
    nav2_param_file_path = PathJoinSubstitution([this_package, 'param', 'mini_pupper.yaml'])
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    rviz_config_file_path = PathJoinSubstitution([this_package, 'rviz', 'navigation.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    map = LaunchConfiguration('map')
    map_launch_arg = DeclareLaunchArgument(
        name='map',
        default_value=default_map_path,
        description='Full path to map file to load'
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        map_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': map,
                'params_file': nav2_param_file_path,
                'use_sim_time': use_sim_time
            }.items()
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', rviz_config_file_path
            ],
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),
        # Uncomment the following to Log map path for debugging
        # LogInfo(msg=map),
    ])
