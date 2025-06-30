from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dance_node = Node(
            package="mini_pupper_dance",
            namespace="",
            executable="mini_pupper_dance",
            name="mini_pupper_dance",
        )
    return LaunchDescription([
        dance_node
    ])
