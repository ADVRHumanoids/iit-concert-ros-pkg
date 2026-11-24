import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('concert_gazebo')
    examples = get_package_share_directory('concert_examples')
    return LaunchDescription([
        DeclareLaunchArgument('modular_description', default_value=os.path.join(examples, 'pino_6dof.py')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'modular.launch.py'))),
    ])
