#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
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
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_pupper_tracking',
            executable='webcam_node',
            name='mini_pupper_webcam_node',
            output='screen',
        ),
        Node(
            package='mini_pupper_tracking',
            executable='main',
            name='mini_pupper_tracking_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('mini_pupper_tracking'),
                    'config', 'tracking_params.yaml'
                )
            ],
            output='screen'
        ),
        Node(
            package='mini_pupper_tracking',
            executable='movement_node',
            name='mini_pupper_movement_node',
            parameters=[
                os.path.join(
                    get_package_share_directory('mini_pupper_tracking'),
                    'config', 'movement_params.yaml'
                )
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/robot_command'],
            name='robot_command_monitor',
            output='screen'
        )
    ])

