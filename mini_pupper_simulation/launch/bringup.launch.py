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

# Upgraded from ROS2 Humble to ROS2 Jazzy

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mini_pupper_simulation = get_package_share_directory('mini_pupper_simulation')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mini_pupper_simulation, 'launch', 'gazebo.launch.py')
        )
    )

    sim_ros2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_mini_pupper_simulation, 'launch', 'sim_ros2_controllers.launch.py'
            )
        )
    )

    return LaunchDescription([
        gazebo_launch,
        sim_ros2_controllers_launch,
    ])
