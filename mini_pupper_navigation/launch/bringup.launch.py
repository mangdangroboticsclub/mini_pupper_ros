#!/usr/bin/env python3
#
# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2023 MangDang
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

# This program is based on https://github.com/ros-planning/navigation2
# which are released under the Apache-2.0 License.
# http://www.apache.org/licenses/LICENSE-2.0
#
# Copyright (c) 2018 Intel Corporation
#
# https://github.com/ros-planning/navigation2/blob/6d12726efb936797b527edfa2f3191ae52f9ae77/nav2_bringup/launch/bringup_launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('mini_pupper_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    # map_yaml_file = LaunchConfiguration('map') # not used for now
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    resolution = LaunchConfiguration('resolution')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    publish_period_sec = LaunchConfiguration('publish_period_sec')
    slam_configuration_basename = LaunchConfiguration(
        'slam_configuration_basename')
    nav_configuration_basename = LaunchConfiguration(
        'nav_configuration_basename')
    load_state_filename = LaunchConfiguration('load_state_filename')
    imu_enable = LaunchConfiguration('imu_enable')
    rviz = LaunchConfiguration('rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            launch_dir, 'maps', 'cartographer_map.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            bringup_dir, 'config', 'nav2', 'mini_pupper.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution', default_value='0.05',
        description='Resolution for Cartographer'
    )

    declare_cartographer_config_dir = DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=os.path.join(bringup_dir, 'config', 'cartographer'),
        description='Cartographer config directory path'
    )

    declare_slam_configuration_basename = DeclareLaunchArgument(
        'slam_configuration_basename', default_value='slam.lua',
        description='SLAM Cartographer configuation file under "cartographer_config_dir"'
    )

    declare_publish_period_sec = DeclareLaunchArgument(
        'publish_period_sec', default_value='1.0',
        description='Cartographer OccupancyGrid publishing period'
    )

    declare_nav_configuration_basename = DeclareLaunchArgument(
        'nav_configuration_basename', default_value='nav.lua',
        description='Localization Cartographer configuation file under "cartographer_config_dir"'
    )

    declare_load_state_filename = DeclareLaunchArgument(
        'load_state_filename',
        default_value=os.path.join(bringup_dir, 'maps', 'cartographer_map.pbstream'),
        description='Map state file path load for cartographer localization'
    )

    declare_imu_enable = DeclareLaunchArgument(
        'imu_enable', default_value='true',
        description='Enable IMU sensor'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'mini_pupper_nav.rviz'),
        description='Full path to the RViz config file to use'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        Node(
            name='imu_filter',
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(imu_enable),
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'slam.launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'use_sim_time': use_sim_time,
                              'cartographer_config_dir': cartographer_config_dir,
                              'configuration_basename': slam_configuration_basename,
                              'publish_period_sec': publish_period_sec,
                              'resolution': resolution}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization.launch.py')),
            condition=UnlessCondition(slam),
            launch_arguments={'use_sim_time': use_sim_time,
                              'cartographer_config_dir': cartographer_config_dir,
                              'configuration_basename': nav_configuration_basename,
                              'load_state_filename': load_state_filename,
                              'publish_period_sec': publish_period_sec,
                              'resolution': resolution}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, 'navigation.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
            output='screen')
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_cartographer_config_dir)
    ld.add_action(declare_publish_period_sec)
    ld.add_action(declare_slam_configuration_basename)
    ld.add_action(declare_nav_configuration_basename)
    ld.add_action(declare_load_state_filename)
    ld.add_action(declare_imu_enable)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
