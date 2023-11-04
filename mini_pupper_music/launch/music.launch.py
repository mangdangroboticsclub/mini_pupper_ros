from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    music_server_node = Node(
            package="mini_pupper_music",
            namespace="",
            executable="service",
            name="music_server",
        )
    return LaunchDescription([
        music_server_node
    ])
