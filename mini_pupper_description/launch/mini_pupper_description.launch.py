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

import os

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    ROBOT_MODEL = os.getenv("ROBOT_MODEL", default="mini_pupper_2")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name="use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true"
    )

    use_debug_stand = LaunchConfiguration("use_debug_stand")
    use_debug_stand_launch_arg = DeclareLaunchArgument(
        name="use_debug_stand", default_value="false",
        description="Include debug support stand in URDF"
    )

    use_gazebo_hardware = LaunchConfiguration("use_gazebo_hardware")
    use_gazebo_hardware_launch_arg = DeclareLaunchArgument(
        name="use_gazebo_hardware", default_value="false",
        description="Use Gazebo hardware interface if true"
    )

    default_model_path = PathJoinSubstitution([
        FindPackageShare("mini_pupper_description"),
        "urdf",
        ROBOT_MODEL,
        "mini_pupper_description.urdf.xacro"
    ])

    description_path = LaunchConfiguration("description_path")
    description_path_launch_arg = DeclareLaunchArgument(
        name="description_path",
        default_value=default_model_path,
        description="Absolute path to robot urdf file")
    robot_description_content = Command(
        [
            "xacro ",
            description_path,
            " ",
            "use_debug_stand:=",
            use_debug_stand,
            " ",
            "use_gazebo_hardware:=",
            use_gazebo_hardware,
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription(
        [
            description_path_launch_arg,
            use_sim_time_launch_arg,
            use_debug_stand_launch_arg,
            use_gazebo_hardware_launch_arg,
            robot_state_publisher_node,
        ]
    )
