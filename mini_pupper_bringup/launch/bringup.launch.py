# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_bringup')
    hardware_connected = LaunchConfiguration("hardware_connected", default='false')

    joints_config = PathJoinSubstitution(
        [this_package, 'config', 'joints', 'joints.yaml']
    )
    links_config = PathJoinSubstitution(
        [this_package, 'config', 'links', 'links.yaml']
    )
    gait_config = PathJoinSubstitution(
        [this_package, 'config', 'gait', 'gait.yaml']
    )
    description_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_description'), 'urdf', 'mini_pupper_description.urdf.xacro']
    )
    champ_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py']
    )
    servo_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_control'), 'launch', 'servo_interface.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='robot_name', 
            default_value='',
            description='Set robot name for multi robot'
        ),

        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='hardware_connected', 
            default_value=hardware_connected,
            description='Set to true if connected to a physical robot'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(champ_bringup_launch_path),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("sim"),
                "robot_name": LaunchConfiguration("robot_name"),
                "gazebo": LaunchConfiguration("sim"),
                "rviz": LaunchConfiguration("rviz"),
                "hardware_connected": LaunchConfiguration("hardware_connected"),
                "publish_foot_contacts": "true",
                "close_loop_odom": "true",
                "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                "joints_map_path": joints_config,
                "links_map_path": links_config,
                "gait_config_path": gait_config,
                "description_path": description_path
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(servo_bringup_launch_path),
            launch_arguments={}.items(),
            condition = IfCondition(hardware_connected),
        ),

        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='LD06',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD06'},
                {'topic_name': 'scan'},
                {'frame_id': 'base_laser'},
                {'port_name': '/dev/ttyUSB0'},
                {'port_baudrate': 230400},
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False},
                {'angle_crop_min': 135.0},
                {'angle_crop_max': 225.0}],
            condition = IfCondition(hardware_connected)
        ),
  
        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            name = 'base_link_to_base_laser_ld06',
            arguments = ['0', '0', '0', '1.57', '0', '0', 'base_link', 'base_laser'],
            condition = IfCondition(hardware_connected),
        ),
    ])
