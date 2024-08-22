#!/usr/bin/env python3

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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    pid_arg = DeclareLaunchArgument(
        name='pid',
        default_value='False',
        description='Enable pid if true'
    )

    line_detection_node = Node(
        package="mini_pupper_recognition",
        namespace="",
        executable="line_detection_node",
        name="line_detection_node",
    )

    pid_line_following_node = Node(
        package="mini_pupper_recognition",
        namespace="",
        executable="pid_line_following_node",
        name="pid_line_following_node",
        condition=IfCondition(pid_arg),
    )

    normal_line_following_node = Node(
        package="mini_pupper_recognition",
        namespace="",
        executable="normal_line_following_node",
        name="normal_line_following_node",
        condition=IfCondition(~pid_arg),
    )

    return LaunchDescription([
        pid_arg,
        line_detection_node,
        pid_line_following_node,
        normal_line_following_node
    ])
