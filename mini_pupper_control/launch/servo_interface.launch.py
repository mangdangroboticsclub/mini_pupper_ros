from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mini_pupper_control',
            executable='servo_interface',
            name='servo_interface',
            output='screen'
        )
    ])
