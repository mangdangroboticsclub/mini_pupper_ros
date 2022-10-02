
import os

import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true")

    default_model_path =  PathJoinSubstitution(
        [FindPackageShare('mini_pupper_description'), 'urdf', 'mini_pupper_description.urdf.xacro']
    )    

    declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        
        parameters=[
            {"robot_description": Command(["xacro ", description_path])},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription(
        [
            declare_description_path,
            declare_use_sim_time,
            robot_state_publisher_node,
        ]
    )