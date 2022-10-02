import os
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
    
def generate_launch_description():
    gazebo_package = FindPackageShare('mini_pupper_gazebo')
    bringup_package = FindPackageShare('mini_pupper_bringup')

    joints_config = PathJoinSubstitution(
        [bringup_package, 'config', 'joints', 'joints.yaml']
    )
    links_config = PathJoinSubstitution(
        [bringup_package, 'config', 'links', 'links.yaml']
    )
    gait_config = PathJoinSubstitution(
        [bringup_package, 'config', 'gait', 'gait.yaml']
    )
    ros_control_config = PathJoinSubstitution(
        [gazebo_package, 'config', 'ros_control', 'ros_control.yaml']
    )
    description_path = PathJoinSubstitution(
        [FindPackageShare('mini_pupper_description'), 'urdf', 'mini_pupper_description.urdf.xacro']
    )
    bringup_launch_path = PathJoinSubstitution(
        [FindPackageShare('champ_bringup'), 'launch', 'bringup.launch.py']
    )
    default_world_path = PathJoinSubstitution(
        [gazebo_package, 'worlds', 'playground.world']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            name="rviz", 
            default_value="false", 
            description="Launch rviz"
        ),
        
        DeclareLaunchArgument(
            name="robot_name",
            default_value="",
            description="Robot name"
        ),
        
        DeclareLaunchArgument(
            name="lite", 
            default_value="false", 
            description="Lite"
        ),
        
        DeclareLaunchArgument(
            name="ros_control_file",
            default_value=ros_control_config,
            description="Ros control config path",
        ),
        
        DeclareLaunchArgument(
            name="world", 
            default_value=default_world_path, 
            description="Gazebo world name"
        ),
        
        DeclareLaunchArgument(
            name="gui", 
            default_value="true", 
            description="Use gui"
        ),
        
        DeclareLaunchArgument(
            name="world_init_x", 
            default_value="0.0"
        ),
        
        DeclareLaunchArgument(
            name="world_init_y", 
            default_value="0.0"
        ),
        
        DeclareLaunchArgument(
            name="world_init_heading", 
            default_value="0.6"
        ),
        
        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),
        
        DeclareLaunchArgument(
            name='hardware_connected', 
            default_value='false',
            description='Set to true if connected to a physical robot'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
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
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("champ_gazebo"),"launch","gazebo.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_name": LaunchConfiguration("robot_name"),
                "world": LaunchConfiguration("world"),
                "lite": LaunchConfiguration("lite"),
                "world_init_x": LaunchConfiguration("world_init_x"),
                "world_init_y": LaunchConfiguration("world_init_y"),
                "world_init_heading": LaunchConfiguration("world_init_heading"),
                "gui": LaunchConfiguration("gui"),
                "close_loop_odom": "true",
            }.items(),
        )
    ])
