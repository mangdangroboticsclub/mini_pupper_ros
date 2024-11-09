from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    this_package = FindPackageShare('mini_pupper_simulation')

    default_world = PathJoinSubstitution([this_package, 'worlds', 'mini_pupper_home.world'])

    world = LaunchConfiguration('world')
    world_launch_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo world path'
    )

    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])
    gazebo_params_path = PathJoinSubstitution([this_package, 'config', 'gazebo_params.yaml'])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'extra_gazebo_args': f'--ros-args --params-file \{gazebo_params_path}',
            'world': world
        }.items()
    )

    return LaunchDescription([
        world_launch_arg,
        gazebo_launch
    ])
