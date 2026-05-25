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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")

    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot1",  # or whatever default you want
        description="Namespace for this robot"
    )
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
        namespace=robot_namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", ["/", robot_namespace, "/controller_manager"],
            "--param-file", controller_params_file
        ],
        output="screen",
    )

    simple_quadruped_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_namespace,
        arguments=[
            "simple_quadruped_controller",
            "--controller-manager", ["/", robot_namespace, "/controller_manager"],
            "--param-file", controller_params_file
        ],
        output="screen",
    )

    #it doesn't matter if the spawner has namespace or not (suppose should have when it launches multiple robot -> conflict)
    #The problem lies at the fact that the mini_pupper_2_controllers.yaml must match with the namespace, for example : 
    """
    robot1:
        controller_manager:
            ros__parameters:
            use_sim_time: False
            update_rate: 100  # Hz (realistic for servo hardware, still >67Hz for smooth interpolation)
            joint_state_broadcaster:
                type: joint_state_broadcaster/JointStateBroadcaster
            simple_quadruped_controller:
                type: mini_pupper_controllers/SimpleQuadrupedController

        simple_quadruped_controller:
            ros__parameters:
    """

    # Start the broadcaster once controller_manager is up, then load the
    # quadruped controller after the broadcaster spawner completes.
    joint_state_broadcaster_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner],
            #namespace=robot_namespace
        )
    )

    quadruped_controller_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[simple_quadruped_controller_spawner],
            #namespace= 
        )
    )

    return LaunchDescription([
        controller_manager_node,
        joint_state_broadcaster_handler,
        quadruped_controller_handler,
    ])


#Finish for multi_robot case
#only work: fix the mode multi-robot. Currently the mode multi-robot is applied to single launch and multi-robot launch
#What changed did i made : ? I add the param file for the spawner under the namespace, and changed the param file mini_pupper_2_controllers.yaml to wildcare
#source: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html
