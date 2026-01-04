#!/usr/bin/env python3
#
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
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command(["ros2 param get --hide-type /robot_state_publisher robot_description"])

    controller_params_file = PathJoinSubstitution([
        FindPackageShare("mini_pupper_description"),
        "config",
        "ros2_control",
        "mini_pupper_2_controllers.yaml"
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": False},
                    controller_params_file],
        output="screen",
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager_node])

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

    delayed_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner, simple_quadruped_controller_spawner],
        )
    )

    return LaunchDescription([
        delayed_controller_manager,
        delayed_controllers_spawner,
    ])
