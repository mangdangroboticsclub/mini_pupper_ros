#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 MangDang
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# This program is based on https://github.com/champ/champ.
# which is released under the BSD-3-Clause License.
# https://spdx.org/licenses/BSD-3-Clause.html
#
# Copyright (c) 2019-2020 Juan Miguel Jimeno
#
# https://github.com/chvmp/champ/blob/f76d066d8964c8286afbcd9d5d2c08d781e85f54/champ_description/launch/description.launch.py

import os
import launch_ros
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
