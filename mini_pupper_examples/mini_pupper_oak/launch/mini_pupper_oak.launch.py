import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'mini_pupper_oak',
            executable = 'oak_detect',
            name = 'mini_pupper_oak'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('depthai_examples'), 'launch', 'mobile_publisher.launch.py')),
            launch_arguments={}.items(),
        ),
    ])
