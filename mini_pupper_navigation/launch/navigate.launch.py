import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default = os.path.join( get_package_share_directory('mini_pupper_navigation'), 'config/cartographer'))
                                                  
    configuration_basename = LaunchConfiguration('configuration_basename', default = 'nav.lua')
    
    load_state_filename = LaunchConfiguration('load_state_filename', default = os.path.join(get_package_share_directory('mini_pupper_navigation'), 'maps', 'mymap.pbstream'))
                                                 
    resolution = LaunchConfiguration('resolution', default = '0.05')
    
    publish_period_sec = LaunchConfiguration('publish_period_sec', default = '1.0')
    
    rviz_config_dir = os.path.join(get_package_share_directory('mini_pupper_navigation'), 'rviz', 'cartographer.rviz')

    return LaunchDescription([
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time},
        #                 {'yaml_filename': map_file}]),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value = use_sim_time,
            description = 'use_sim_time'),
        
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value = cartographer_config_dir,
            description = 'Full path to config file to load'),
        
        DeclareLaunchArgument(
            'configuration_basename',
            default_value = configuration_basename,
            description = 'Name of lua file for cartographer'),
        
        DeclareLaunchArgument(
            'load_state_filename',
            default_value = load_state_filename,
            description = 'pbstream file'),
        
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_node',
            name = 'cartographer_node',
            output = 'screen',
            parameters = [{'use_sim_time': use_sim_time}],
            arguments = [
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', load_state_filename,
            ]),
        
        DeclareLaunchArgument(
            'resolution',
            default_value = resolution,
            description = 'resolution'),
        
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value = publish_period_sec,
            description = 'publish_period_sec'),
        
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': use_sim_time},
                {'-resolution': resolution},
                {'-publish_period_sec': publish_period_sec}]),
        
        # Node(
        #     package = 'nav2_lifecycle_manager',
        #     executable = 'lifecycle_manager',
        #     name = 'nav2_lifecycle_manager',
        #     output = 'screen',
        #     parameters = [{'use_sim_time': use_sim_time},
        #                 {'autostart': True},
        #                 {'node_names': ['map_server']}]),
        
        Node(
            package = 'rviz2',
            executable = 'rviz2',
            name = 'rviz2',
            arguments = ['-d', rviz_config_dir],
            parameters = [{'use_sim_time': use_sim_time}],
            output = 'screen'),
    ])
