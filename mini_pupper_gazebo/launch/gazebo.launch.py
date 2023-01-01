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
    gazebo_package = get_package_share_directory('mini_pupper_gazebo')
    bringup_package = get_package_share_directory('mini_pupper_bringup')
    description_package = get_package_share_directory('mini_pupper_description')

    joints_config_path = os.path.join(
        description_package, 'config', 'joints', 'joints.yaml')
    links_config_path = os.path.join(
        description_package, 'config', 'links', 'links.yaml')
    gait_config_path = os.path.join(
        description_package, 'config', 'gait', 'gait.yaml')
    ros_control_config_path = os.path.join(
        description_package, 'config', 'mini_pupper_controller.yaml')
    description_path = os.path.join(
        description_package, 'urdf', 'mini_pupper_description.urdf.xacro')
    default_world_path = os.path.join(
        gazebo_package, 'worlds', 'default.world')

    bringup_path = os.path.join(
        bringup_package, 'launch', 'bringup.launch.py')

    champ_bringup_path = os.path.join(
        get_package_share_directory('champ_bringup'),
        'launch',
        'bringup.launch.py'
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
            default_value=ros_control_config_path,
            description="ROS control config path",
        ),

        DeclareLaunchArgument(
            name="world",
            default_value=default_world_path,
            description="Gazebo world path"
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
            PythonLaunchDescriptionSource(champ_bringup_path),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("sim"),
                "robot_name": LaunchConfiguration("robot_name"),
                "gazebo": LaunchConfiguration("sim"),
                "rviz": LaunchConfiguration("rviz"),
                "hardware_connected": LaunchConfiguration("hardware_connected"),
                "publish_foot_contacts": "true",
                "close_loop_odom": "true",
                "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
                "joints_map_path": joints_config_path,
                "links_map_path": links_config_path,
                "gait_config_path": gait_config_path,
                "description_path": description_path,
                "ros_control_file": ros_control_config_path
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("champ_gazebo"), "launch", "gazebo.launch.py")),
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
