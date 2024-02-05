#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022 MangDang
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

# This program is based on https://github.com/champ/champ.
# which is released under the BSD-3-Clause License.
# https://spdx.org/licenses/BSD-3-Clause.html
#
# Copyright (c) 2019-2020 Juan Miguel Jimeno
#
# https://github.com/chvmp/champ/blob/f76d066d8964c8286afbcd9d5d2c08d781e85f54/champ_description/launch/description.launch.py

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true")

    default_model_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_description'), 'urdf', 'mini_pupper_description.urdf.xacro']
    )

    declare_description_path = DeclareLaunchArgument(
        name="description_path", default_value=default_model_path,
        description="Absolute path to robot urdf file")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",

        parameters=[
            {"robot_description": Command(["xacro ", description_path])},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription(
        [
            declare_description_path,
            declare_use_sim_time,
            robot_state_publisher_node,
        ]
    )
