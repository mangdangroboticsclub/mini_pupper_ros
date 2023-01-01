from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_pupper_control',
            executable='display_interface',
            name='display_interface',
            output='screen'
        )
    ])
