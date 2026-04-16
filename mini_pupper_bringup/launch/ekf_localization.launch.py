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


from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_package = FindPackageShare("mini_pupper_bringup")
    baselink_to_odom_ekf_config_path = PathJoinSubstitution(
        [bringup_package, "config", "ekf", "baselink_to_odom.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Single EKF: fuses IMU heading to publish odom→base_footprint TF and /odom.
    # base_footprint→base_link is provided as a fixed joint by robot_state_publisher
    # (defined in the URDF), so the old base_to_footprint_ekf is no longer needed.
    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="baselink_to_odom_ekf",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            baselink_to_odom_ekf_config_path,
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    return LaunchDescription(
        [use_sim_time_launch_arg, footprint_to_odom_ekf]
    )
