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
from launch.conditions import IfCondition


def generate_launch_description():
    has_lidar = LaunchConfiguration("has_lidar")
    has_lidar_launch_arg = DeclareLaunchArgument(
        name='has_lidar',
        description='if the robot has lidar sensor'
    )

    has_imu = LaunchConfiguration("has_imu")
    has_imu_launch_arg = DeclareLaunchArgument(
        name='has_imu',
        description='if the robot has imu sensor'
    )

    driver_package = FindPackageShare('mini_pupper_driver')

    servos_launch_path = PathJoinSubstitution(
        [driver_package, 'launch', 'servo_interface.launch.py']
    )
    lidar_launch_path = PathJoinSubstitution(
        [driver_package, 'launch', 'lidar_ld06.launch.py']
    )
    imu_launch_path = PathJoinSubstitution(
        [driver_package, 'launch', 'imu_interface.launch.py']
    )

    return LaunchDescription([
        has_lidar_launch_arg,
        has_imu_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servos_launch_path)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path),
            condition=IfCondition(has_lidar)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch_path),
            condition=IfCondition(has_imu)
        )
    ])
