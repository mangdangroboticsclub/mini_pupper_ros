# Copyright (c) 2023 Yunlong Feng
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory('mini_pupper_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    configuration_basename = LaunchConfiguration('configuration_basename')
    load_state_filename = LaunchConfiguration('load_state_filename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    rviz_config_dir = os.path.join(get_package_share_directory(
        'mini_pupper_navigation'), 'rviz', 'navigation.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='use_sim_time'),

        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=os.path.join(bringup_dir, 'config', 'cartographer'),
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value='nav.lua',
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'load_state_filename',
            default_value=os.path.join(
                bringup_dir, 'maps', 'cartographer_map.pbstream'),
            description='pbstream file'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', load_state_filename,
            ]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='resolution'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'-resolution': resolution},
                {'-publish_period_sec': publish_period_sec}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    ])
