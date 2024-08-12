from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ai_image_service_node = Node(
            package="node",
            namespace="",
            executable="service",
            name="ai_image_service",
        )
    pose_controller_node = Node(
            package="mini_pupper_dance",
            namespace="",
            executable="pose_controller",
            name="pose_controller",
        )
    return LaunchDescription([
        ai_image_service_node,
        pose_controller_node
    ])