#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2025 MangDang
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

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)

ROBOT_MODEL = os.getenv("ROBOT_MODEL", default="mini_pupper_2")


def get_config():
    bringup_package = get_package_share_directory("mini_pupper_bringup")
    config_file_name = ROBOT_MODEL + ".yaml"
    config_file_path = os.path.join(bringup_package, "config", config_file_name)

    with open(config_file_path, "r") as f:
        configuration = yaml.safe_load(f)

    sensors_config = configuration.get("sensors", {})
    sensors_config.setdefault("lidar", False)
    sensors_config.setdefault("imu", False)
    sensors_config.setdefault("camera", False)

    ports_config = configuration.get("ports", {})

    return sensors_config, ports_config


def generate_launch_description():
    bringup_package = FindPackageShare("mini_pupper_bringup")
    description_package = FindPackageShare("mini_pupper_description")

    sensors_config, ports_config = get_config()

    # Convert bool to str because cannot pass bool directly to launch_arguments.
    has_lidar = str(sensors_config["lidar"])
    has_imu = str(sensors_config["imu"])
    has_camera = str(sensors_config["camera"])
    lidar_port = ports_config["lidar"]

    hardware_connected = LaunchConfiguration("hardware_connected")
    hardware_connected_launch_arg = DeclareLaunchArgument(
        name="hardware_connected",
        default_value="true",
        description="Set to true if connected to a physical robot",
    )

    # multi robot and namespacing
    multi_robot = LaunchConfiguration("multi_robot")
    multi_robot_arg = DeclareLaunchArgument(
        "multi_robot",
        default_value="false",
        description="Enable multi-robot mode with namespacing",
    )

    robot_namespace = LaunchConfiguration("robot_namespace")
    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value=[
            TextSubstitution(text="robot"),
            EnvironmentVariable("ROBOT_ID", default_value="1"),
        ],
        description="Namespace for this robot (e.g. robot1, robot2)",
    )

    description_launch_path = PathJoinSubstitution(
        [description_package, "launch", "mini_pupper_description.launch.py"]
    )
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch_path),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    accessories_launch_path = PathJoinSubstitution(
        [bringup_package, "launch", "accessories.launch.py"]
    )
    accessories_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(accessories_launch_path),
        condition=IfCondition(hardware_connected),
        launch_arguments={
            "has_lidar": has_lidar,
            "has_imu": has_imu,
            "has_camera": has_camera,
            "lidar_port": lidar_port,
        }.items(),
    )

    ros2_controllers_launch_path = PathJoinSubstitution([
        bringup_package,
        "launch",
        "ros2_controllers.launch.py"
    ])
    ros2_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros2_controllers_launch_path)
    )

    stanford_controller_launch_path = PathJoinSubstitution(
        [FindPackageShare("stanford_controller"), "stanford_controller.launch.py"]
    )
    stanford_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stanford_controller_launch_path),
        launch_arguments={
            "orientation_from_imu": has_imu,
            "publish_joint_control": "true",
        }.items(),
    )

    launch_twist_converter = LaunchConfiguration("launch_twist_converter")
    launch_twist_converter_launch_arg = DeclareLaunchArgument(
        name="launch_twist_converter",
        default_value="true",
        description="Launch twist_to_command_converter to convert /cmd_vel to robot_command (set false to use your own pipeline)"
    )

    twist_converter_launch_path = PathJoinSubstitution(
        [FindPackageShare("stanford_controller"), "twist_to_command_converter.launch.py"]
    )
    twist_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_converter_launch_path),
        condition=IfCondition(launch_twist_converter)
    )

    baselink_to_odom_ekf_config_path = PathJoinSubstitution(
        [bringup_package, "config", "ekf", "baselink_to_odom.yaml"]
    )

    # Single EKF: fuses IMU heading to publish odom→base_footprint TF and /odom.
    # base_footprint→base_link is provided as a fixed joint by robot_state_publisher
    # (defined in the URDF), so the old base_to_footprint_ekf is no longer needed.
    footprint_to_odom_ekf_launch = Node(
        package="robot_localization",
        executable="ekf_node",
        name="baselink_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            baselink_to_odom_ekf_config_path,
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    launch_actions = [
        description_launch,
        accessories_launch,
        ros2_controllers_launch,
        stanford_controller_launch,
        twist_converter_launch,
        footprint_to_odom_ekf_launch,
    ]

    launch_description = [
        robot_namespace_arg,
        multi_robot_arg,
        hardware_connected_launch_arg,
        launch_twist_converter_launch_arg,
        GroupAction(
            actions=[PushRosNamespace(robot_namespace)] + launch_actions,
            condition=IfCondition(multi_robot),
        ),
        GroupAction(actions=launch_actions, condition=UnlessCondition(multi_robot)),
    ]

    return LaunchDescription(launch_description)
