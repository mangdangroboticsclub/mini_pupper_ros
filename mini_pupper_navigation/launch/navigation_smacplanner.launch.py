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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
# from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_navigation')

    default_map_path = PathJoinSubstitution([this_package, 'maps', 'map.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2_param_file_path = PathJoinSubstitution([this_package, 'param', 'real_table.yaml'])
    configured_params = RewrittenYaml(
        source_file=nav2_param_file_path,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    rviz_config_file_path = PathJoinSubstitution([this_package, 'rviz', 'navigation.rviz'])

    map_cfg = LaunchConfiguration('map')
    map_launch_arg = DeclareLaunchArgument(
        name='map',
        default_value=default_map_path,
        description='Full path to map file to load'
    )

    # Scope remap to Nav2 only: cmd_vel -> cmd_vel_navigation2
    nav2_group = GroupAction([
        SetRemap(src='cmd_vel', dst='cmd_vel_navigation2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': map_cfg,
                'params_file': configured_params,
                'use_sim_time': use_sim_time,
            }.items()
        ),
    ])

    # Already uses /cmd_vel_navigation2 -> /cmd_vel, so no remaps needed here
    nav_vel_scaler = Node(
        package='mini_pupper_driver',
        executable='nav_vel_scaler',
        name='nav_vel_scaler',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        map_launch_arg,
        nav2_group,
        nav_vel_scaler,
        rviz,
        # LogInfo(msg=map_cfg),
    ])
