import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    ROBOT_MODEL = os.getenv('ROBOT_MODEL', default="mini_pupper_2")

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")

    declare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time", default_value="false",
        description="Use simulation (Gazebo) clock if true")

    default_model_path = PathJoinSubstitution([
        FindPackageShare('mini_pupper_description'),
        'urdf',
        ROBOT_MODEL,
        'mini_pupper_description.urdf.xacro'
    ])

    declare_description_path = DeclareLaunchArgument(
        name="description_path", default_value=default_model_path,
        description="Absolute path to robot urdf file")
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('mini_pupper_description'),
        'rviz',
        'stanford_viewer.rviz'
    ])

    return LaunchDescription(
        [
            declare_description_path,
            declare_use_sim_time,
            Node(
                package='mini_pupper_description',
                executable='stanford_joint_trajectory_to_states',
                name='stanford_joint_trajectory_to_states',
                output='screen',
            ),
            Node(
                package='mini_pupper_description',
                executable='stanford_state_publisher',
                name='stanford_state_publisher',
                output='screen'),
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_path]
            ),
        ]
    )