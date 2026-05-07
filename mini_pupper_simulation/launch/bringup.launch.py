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


import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

ROBOT_MODEL = os.getenv("ROBOT_MODEL", default="mini_pupper_2")


def generate_launch_description():
    this_package = FindPackageShare("mini_pupper_simulation")

    debug_control = LaunchConfiguration("debug_control")
    debug_control_launch_arg = DeclareLaunchArgument(
        name="debug_control",
        default_value="false",
        description="Include support stand in robot description for debugging control (true/false)"
    )

    default_world_path = PathJoinSubstitution([this_package, "worlds", "mini_pupper_home.world"])

    world = LaunchConfiguration("world")
    world_launch_arg = DeclareLaunchArgument(
        name="world",
        default_value=default_world_path,
        description="Gazebo world path"
    )

    gui = LaunchConfiguration("gui")
    gui_launch_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Whether to start the Gazebo GUI"
    )

    # Conditional spawn height based on debug_control
    # When debug stand is enabled, spawn higher since the stand extends below the robot
    # Normal spawn: 0.10m for crouch pose (feet ~7cm below body)
    selected_spawn_z = PythonExpression([
        '"0.396" if "', debug_control, '" == "true" else "0.10"'
    ])

    world_init_z = LaunchConfiguration("world_init_z")
    world_init_z_launch_arg = DeclareLaunchArgument(
        name="world_init_z",
        default_value=selected_spawn_z,
        description="Robot spawn height (higher when debug stand is included in URDF)"
    )

    # Simulation-specific robot description launch with debug stand support
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare("mini_pupper_description"),
         "launch", "mini_pupper_description.launch.py"]
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": "true",
            "use_debug_stand": debug_control,
            "use_gazebo_hardware": "true",
        }.items()
    )

    launch_twist_converter = LaunchConfiguration("launch_twist_converter")
    launch_twist_converter_launch_arg = DeclareLaunchArgument(
        name="launch_twist_converter",
        default_value="true",
        description=(
            "Launch twist_to_command_converter to convert /cmd_vel to "
            "robot_command (set false to use your own pipeline)"
        ),
    )

    twist_converter_launch_path = PathJoinSubstitution(
        [FindPackageShare("stanford_controller"), "twist_to_command_converter.launch.py"]
    )
    twist_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_converter_launch_path),
        condition=IfCondition(launch_twist_converter)
    )

    # Stanford controller launch for simulation
    stanford_controller_launch_path = PathJoinSubstitution(
        [FindPackageShare("stanford_controller"), "stanford_controller.launch.py"]
    )
    stanford_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stanford_controller_launch_path),
        launch_arguments={
            "orientation_from_imu": "false",
            "publish_joint_control": "true",
            "publish_states": "true"
        }.items()
    )

    gazebo_launch_path = PathJoinSubstitution([this_package, "launch", "gazebo.launch.py"])
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            "world": world,
            "gui": gui,
        }.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", ROBOT_MODEL,
            "-x", "0.0",
            "-y", "0.0",
            "-z", world_init_z,
            "-R", "0",
            "-P", "0",
            "-Y", "0.0"
        ],
        output="screen"
    )

    ros2_controllers_launch_path = PathJoinSubstitution([
        this_package,
        "launch",
        "sim_ros2_controllers.launch.py"
    ])
    ros2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_controllers_launch_path)
    )

    # Delayed start: Stanford controller first (wait for controllers to be active)
    delayed_stanford_controller_launch = TimerAction(
        period=8.0,
        actions=[stanford_controller_launch],
    )

    # Delayed start: twist converter after Stanford controller has started
    delayed_twist_converter_launch = TimerAction(
        period=12.0,
        actions=[twist_converter_launch],
    )

    # TF broadcaster for p3d odometry
    odom_tf_broadcaster = Node(
        package="mini_pupper_simulation",
        executable="odom_tf_broadcaster.py",
        name="odom_tf_broadcaster",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        debug_control_launch_arg,
        world_launch_arg,
        gui_launch_arg,
        world_init_z_launch_arg,
        launch_twist_converter_launch_arg,
        description_launch,
        gazebo_launch,
        spawn_entity,
        odom_tf_broadcaster,
        ros2_controllers_launch,
        delayed_stanford_controller_launch,
        delayed_twist_converter_launch,
    ])
