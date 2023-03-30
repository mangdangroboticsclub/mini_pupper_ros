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
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_pupper_control',
            executable='vel_to_servo_controller',
            name='vel_to_servo_controller',
            output='screen'
        ),
<<<<<<< HEAD:mini_pupper_control/launch/vel_to_servo_controller.launch.py
=======
        Node(
            package='mini_pupper_driver',
            executable='servo_interface',
            name='servo_interface',
            output='screen'
        )
>>>>>>> 7f9ceb167a3a6aebf3e5385cecbcf470b883753a:mini_pupper_driver/launch/vel_to_servo_controller.launch.py
    ])


