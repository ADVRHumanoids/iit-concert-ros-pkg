import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap

WHEELS = ['J_wheel_A', 'J_wheel_B', 'J_wheel_C', 'J_wheel_D']


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() == 'true'

    with open(arg('ik'), 'r') as f:
        problem_description = f.read()

    params = {
        'problem_description': problem_description,
        'joint_blacklist': WHEELS,
        'velocity_whitelist': WHEELS,
        'enable_otg': flag('enable_otg'),
        'is_model_floating_base': flag('is_model_floating_base'),
        'model_type': 'pin',
        'solver': 'OpenSot',
        'rate': float(arg('rate')),
        'tf_prefix': arg('tf_prefix'),
    }
    # '!' means unset, as in the ROS1 launch file.
    if arg('world_frame_link') != '!':
        params['world_frame_link'] = arg('world_frame_link')

    remappings = []
    if arg('solution_topic') != '!':
        remappings.append(('cartesian/solution', arg('solution_topic')))

    actions = [PushRosNamespace(arg('namespace'))]
    if flag('xbot'):
        actions += [
            SetRemap('robot_description', 'xbotcore/robot_description'),
            SetRemap('robot_description_semantic', 'xbotcore/robot_description_semantic'),
        ]
    actions.append(Node(
        package='cartesian_interface_ros',
        executable='ros_server_node',
        name='ros_server_node',
        output='screen',
        prefix=arg('prefix') or None,
        parameters=[params],
        remappings=remappings,
    ))
    if flag('markers'):
        actions.append(Node(
            package='cartesian_interface_ros',
            executable='marker_spawner',
            name='interactive_markers',
            output='screen',
            parameters=[{'tf_prefix': arg('tf_prefix')}],
        ))

    return [GroupAction(actions)]


def generate_launch_description():
    share = get_package_share_directory('concert_cartesio')

    args = [
        DeclareLaunchArgument('rate', default_value='100.0'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('ik', default_value=os.path.join(share, 'concert_stack.yaml')),
        DeclareLaunchArgument('xbot', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('tf_prefix', default_value='ci'),
        DeclareLaunchArgument('markers', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='/'),
        DeclareLaunchArgument('is_model_floating_base', default_value='true'),
        DeclareLaunchArgument('world_frame_link', default_value='!'),
        DeclareLaunchArgument('solution_topic', default_value='!'),
        DeclareLaunchArgument('enable_otg', default_value='true'),
    ]

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('gui')),
        package='rviz2',
        executable='rviz2',
        name='ik_rviz',
        arguments=['-d', os.path.join(share, 'rviz', 'concert.rviz')],
    )

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup), rviz])
