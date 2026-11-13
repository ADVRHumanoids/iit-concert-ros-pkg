import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('concert_cartesio')
    return LaunchDescription([
        DeclareLaunchArgument('ik', default_value=os.path.join(share, 'concert_stack_base.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'concert.launch.py'))),
    ])
