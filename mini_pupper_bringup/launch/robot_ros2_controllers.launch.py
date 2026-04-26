#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2026 MangDang
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
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get robot description from URDF file
    robot_model = os.getenv("ROBOT_MODEL", default="mini_pupper_2")
    description_package = FindPackageShare("mini_pupper_description")
    bringup_package = FindPackageShare("mini_pupper_bringup")

    urdf_file = PathJoinSubstitution([
        description_package,
        "urdf",
        robot_model,
        "mini_pupper_description.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file, " use_gazebo_hardware:=false"]),
        value_type=str,
    )

    controller_params_file = PathJoinSubstitution([
        bringup_package,
        "config",
        "ros2_control",
        "mini_pupper_2_controllers.yaml"
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_params_file
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    simple_quadruped_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_quadruped_controller"],
        output="screen"
    )

    # Start the broadcaster once controller_manager is up, then load the
    # quadruped controller after the broadcaster spawner completes.
    joint_state_broadcaster_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    quadruped_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[simple_quadruped_controller_spawner],
        )
    )

    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_handler,
        quadruped_controller_handler,
    ])