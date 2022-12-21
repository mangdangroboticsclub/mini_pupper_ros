import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    map_file = LaunchConfiguration('map_file', default=os.path.join(get_package_share_directory('mini_pupper_navigation'), 'maps', 'mymap.yaml'))
    
    rviz_config_dir = os.path.join(get_package_share_directory('mini_pupper_navigation'), 'rviz', 'cartographer.rviz')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}]),
        
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_node',
            name = 'cartographer_node',
            output = 'screen',
            parameters = [{'use_sim_time': use_sim_time}],
            arguments = [
                '-configuration_directory', os.path.join(get_package_share_directory('mini_pupper_navigation'), 'config/cartographer'),
                '-configuration_basename', 'nav.lua',
                '-load_state_filename', os.path.join(get_package_share_directory('mini_pupper_navigation'), 'maps', 'mymap.pbstream')]),
        
        Node(
            package = 'cartographer_ros',
            executable = 'cartographer_occupancy_grid_node',
            parameters = [
                {'use_sim_time': use_sim_time},
                {'resolution': 0.05}]),
        
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='nav2_lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
