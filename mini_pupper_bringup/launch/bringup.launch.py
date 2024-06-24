#!/usr/bin/env python3

# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2022-2023 MangDang
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

ROBOT_MODEL = os.getenv('ROBOT_MODEL', default="mini_pupper_2")

def get_config():
    bringup_package = get_package_share_directory('mini_pupper_bringup')
    config_file_name = ROBOT_MODEL + '.yaml'
    config_file_path = os.path.join(bringup_package, 'config', config_file_name)

    with open(config_file_path, 'r') as f:
        configuration = yaml.safe_load(f)

    sensors_config = configuration.get('sensors', {})
    sensors_config.setdefault('lidar', False)
    sensors_config.setdefault('imu', False)

    ports_config = configuration.get('ports', {})

    return sensors_config, ports_config

def generate_launch_description():

    description_package = FindPackageShare('mini_pupper_description')

    description_path = PathJoinSubstitution(
        [description_package, 'urdf', ROBOT_MODEL , + 'mini_pupper_description.urdf.xacro']
    )

    joints_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', ROBOT_MODEL, 'joints.yaml']
    )
    links_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', ROBOT_MODEL, 'links.yaml']
    )
    gait_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', ROBOT_MODEL, 'gait.yaml']
    )

    champ_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py']
    )
    hardware_interface_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_bringup'), 'launch', 'hardware_interface.launch.py']
    )

    sensors_config, ports_config = get_config()

    # This is the confusing part to wrap so much around a bool value.
    # In ROS2 launch file, we cannot pass bool value directly to launch_arguments.
    has_lidar = PythonExpression([str(sensors_config['lidar'])])
    has_imu = PythonExpression([str(sensors_config['imu'])])
    lidar_port = PythonExpression([str(ports_config['lidar'])])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_launch_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    hardware_connected = LaunchConfiguration("hardware_connected")
    hardware_connected_launch_arg = DeclareLaunchArgument(
        name='hardware_connected',
        default_value='True',
        description='Set to true if connected to a physical robot'
    )

    champ_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(champ_bringup_launch_path),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": ROBOT_MODEL,
            "gazebo": use_sim_time,
            "rviz": "False",  # set always false to launch RViz2 with costom .rviz file
            "joint_hardware_connected": hardware_connected,
            "orientation_from_imu": has_imu,
            "publish_foot_contacts": "True",
            "close_loop_odom": "True",
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "joints_map_path": joints_config_path,
            "links_map_path": links_config_path,
            "gait_config_path": gait_config_path,
            "description_path": description_path
        }.items()
    )

    hardware_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_interface_launch_path),
        condition=IfCondition(hardware_connected),
        launch_arguments={
            "has_lidar": has_lidar,
            "has_imu": has_imu,
            "lidar_port" : lidar_port
        }.items()
    )

    return LaunchDescription([
        use_sim_time_launch_arg,
        hardware_connected_launch_arg,
        champ_bringup_launch,
        hardware_interface_launch
    ])
