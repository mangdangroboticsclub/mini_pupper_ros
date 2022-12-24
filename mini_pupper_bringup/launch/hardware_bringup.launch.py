import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    servo_bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_control'), 'launch', 'servo_interface.launch.py']
    )

    return LaunchDescription([
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
