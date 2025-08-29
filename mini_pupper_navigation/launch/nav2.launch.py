import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mini_pupper_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    default_params_file = os.path.join(package_dir, 'param', 'nav2_params.yaml')
    default_map_file = os.path.join(package_dir, 'maps', 'map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the ROS2 parameters file to use'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file to load'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Use composed bringup if True'
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': LaunchConfiguration('params_file'),
                'map': LaunchConfiguration('map'),
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': LaunchConfiguration('use_composition'),
                'namespace': LaunchConfiguration('namespace'),
                'slam': 'False',
            }.items(),
        ),
    ])