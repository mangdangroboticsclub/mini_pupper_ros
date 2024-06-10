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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

ROBOT_MODEL = os.getenv('ROBOT_MODEL', default="mini_pupper_2")


def get_sensors_config():
    bringup_package = get_package_share_directory('mini_pupper_bringup')
    config_file_name = ROBOT_MODEL + '.yaml'
    config_file_path = os.path.join( bringup_package, 'config', config_file_name)

    with open(config_file_path, 'r') as f:
        configuration = yaml.safe_load(f)
        return configuration['sensors']


def generate_launch_description():
    if ROBOT_MODEL == "mini_pupper_2":
        description_package = FindPackageShare('mini_pupper_2_description')
    else:
        description_package = FindPackageShare('mini_pupper_description')
    
    description_path = PathJoinSubstitution(
        [description_package, 'urdf', 'mini_pupper_description.urdf.xacro']
    )

    joints_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', 'joints.yaml']
    )
    links_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', 'links.yaml']
    )
    gait_config_path = PathJoinSubstitution(
        [description_package, 'config', 'champ', 'gait.yaml']
    )

    champ_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py']
    )
    hardware_interface_launch_path =  PathJoinSubstitution(
        [FindPackageShare('mini_pupper_bringup'), 'launch', 'hardware_interface.launch.py']
    )

    simulation = LaunchConfiguration("simulation")
    simulation_launch_arg = DeclareLaunchArgument(
        name='simulation',
        default_value='False',
        description='In Gazebo simulation, no need to launch hardware sensors'
    )

    is_real_robot = PythonExpression(['not ', LaunchConfiguration('simulation')])
    sensors_config = get_sensors_config()
    has_lidar = TextSubstitution(text=str(sensors_config['lidar']))
    has_imu = TextSubstitution(text=str(sensors_config['imu']))
    print(sensors_config)
    return LaunchDescription([
        simulation_launch_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(champ_bringup_launch_path),
            launch_arguments={
                "use_sim_time": simulation,
                "robot_name": ROBOT_MODEL,
                "gazebo": simulation,
                "rviz": "False",  # set always false to launch RViz2 with costom .rviz file
                "joint_hardware_connected": is_real_robot,
                "orientation_from_imu": has_imu,
                "publish_foot_contacts": "True",
                "close_loop_odom": "True",
                "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                "joints_map_path": joints_config_path,
                "links_map_path": links_config_path,
                "gait_config_path": gait_config_path,
                "description_path": description_path
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_interface_launch_path),
            condition=IfCondition(is_real_robot),
            launch_arguments={
                "has_lidar": has_lidar,
                "has_imu": has_imu
            }.items()
        )
    ])
