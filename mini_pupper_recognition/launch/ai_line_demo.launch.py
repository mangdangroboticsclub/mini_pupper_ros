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


def generate_launch_description():
    cloud_line_recognition_node = Node(
            package="mini_pupper_recognition",
            namespace="",
            executable="cloud_line_recognition_node",
            name="cloud_line_recognition_node",
        )
    cloud_line_following_node = Node(
            package="mini_pupper_recognition",
            namespace="",
            executable="cloud_line_following_node",
            name="cloud_line_following_node",
        )
    return LaunchDescription([
        cloud_line_recognition_node,
        cloud_line_following_node
    ])