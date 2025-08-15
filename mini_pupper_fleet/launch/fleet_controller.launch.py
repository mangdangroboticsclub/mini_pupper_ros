# !/usr/bin/env python3
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    count = max(1, int(LaunchConfiguration('robot_count').perform(context)))

    nodes = []

    # Fleet controller with robot count parameter
    nodes.append(
        Node(
            package='mini_pupper_fleet',
            executable='fleet_controller_node',
            name='fleet_controller_node',
            output='screen',
            parameters=[
                {'robot_count': count}
            ]
        )
    )
    
    # Always launch with namespaces (fleet architecture)
    for i in range(1, count + 1):

        nodes.append(
            Node(
                package='mini_pupper_fleet',
                executable='imu_ekf_node',
                name='imu_ekf_node',
                namespace=f'robot{i}',
                output='screen',
            )
        )

        nodes.append(
            Node(
                package='mini_pupper_fleet',
                executable='robot_behaviour_node',
                name='robot_behaviour_node',
                namespace=f'robot{i}',
                output='screen',
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_count', default_value='1',
                              description='Number of robots in the fleet'),
        OpaqueFunction(function=_launch_nodes),
    ])
