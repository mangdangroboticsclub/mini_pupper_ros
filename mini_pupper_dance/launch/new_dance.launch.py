from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_nodes(context, *args, **kwargs):
    multi = (LaunchConfiguration('multi_robot').perform(context).lower() == 'true')
    count = max(1, int(LaunchConfiguration('robot_count').perform(context)))

    nodes = []

    if multi:
        # Launch multiple nodes with remapped topics
        for i in range(1, count + 1):
            nodes.append(
                Node(
                    package='mini_pupper_dance',
                    executable='mini_pupper_dance',
                    name=f'mini_pupper_dance_{i}', # unique node name
                    output='screen',
                    remappings=[
                        # remap the node’s internal topic "robot_command"
                        ('robot_command', f'/robot{i}/robot_command'),
                    ],
                )
            )
    else:
        nodes.append(
            Node(
                package='mini_pupper_dance',
                executable='mini_pupper_dance',
                name='mini_pupper_dance',
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('multi_robot', default_value='false',
                              description='Set true to launch multiple robots'),
        DeclareLaunchArgument('robot_count', default_value='1',
                              description='Number of robots when multi_robot is true'),
        OpaqueFunction(function=_launch_nodes),
    ])
