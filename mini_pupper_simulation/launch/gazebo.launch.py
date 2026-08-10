# Copyright 2024 MangDang
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

# Upgraded from ROS2 Humble (gazebo_ros) to ROS2 Jazzy (gz_ros2_control / gz-sim)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mini_pupper_simulation = get_package_share_directory('mini_pupper_simulation')

    world = LaunchConfiguration('world')
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_mini_pupper_simulation, 'worlds', 'empty.world'),
        description='Path to world file'
    )

    # Use gz sim (Gazebo Harmonic) for Jazzy instead of gazebo_ros (Gazebo Classic) for Humble
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': world}.items()
    )

    return LaunchDescription([
        declare_world_arg,
        gz_sim,
    ])
