"""ROS2 launch for CartesIO arm IK on Concert in Isaac Lab.

Pattern mirrors kyon_cartesio/launch/kyon.launch but ported to ROS2 jazzy.
Plugin/binary names verified against the live image (concert-xbot2):
    package: cartesian_interface_ros
    exec:    ros_server_node, marker_spawner

Why ROS-side and not an xbot2 RT plugin:
    The private xbot2 plugin `albero_cartesio_rt` is not available in the
    current concert-xbot2 image (libxbotctrl_albero_cartesio_rt.so missing,
    its recipe pulls private repos that need credentials in this environment).
    The public `cartesian_interface_ros` ROS server *is* installed in the
    image, so we run cartesio as a separate ROS2 node next to xbot2 and
    let xbot2's `ros2_control` bridge forward joint commands.

Bringup order (three terminals, all inside concert-xbot2 container):
    1) Isaac:  isaaclab -p /workspace/concert_isaac/run_sim.py --enable_cameras
    2) XBot2:  xbot2-core -C ModularBot_isaac_cartesio.yaml -H isaac
               (this publishes /xbotcore/robot_description{,_semantic})
    3) Cartesio:
       ros2 launch concert_cartesio concert_isaac_cartesio.launch.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_nodes(context, *args, **kwargs):
    ik_path = LaunchConfiguration('ik').perform(context)
    problem_yaml = Path(ik_path).read_text()

    rate = LaunchConfiguration('rate')
    tf_prefix = LaunchConfiguration('tf_prefix')
    enable_otg = LaunchConfiguration('enable_otg')
    is_floating_base = LaunchConfiguration('is_floating_base')
    model_type = LaunchConfiguration('model_type')
    solver = LaunchConfiguration('solver')
    use_xbot_topics = LaunchConfiguration('use_xbot_topics').perform(context)

    remappings = []
    if use_xbot_topics.lower() in ('true', '1', 'yes'):
        remappings = [
            ('/robot_description', '/xbotcore/robot_description'),
            ('/robot_description_semantic',
             '/xbotcore/robot_description_semantic'),
        ]

    cartesio_server = Node(
        package='cartesian_interface_ros',
        executable='ros_server_node',
        name='ros_server_node',
        output='screen',
        parameters=[{
            # ros_server_node expects the full YAML content, not a path.
            'problem_description': problem_yaml,
            'rate': rate,
            'tf_prefix': tf_prefix,
            'enable_otg': enable_otg,
            'is_model_floating_base': is_floating_base,
            'model_type': model_type,
            'solver': solver,
            'use_xbot_config': False,
            # `home` is the default group_state name in the SRDF.
            'home': 'home',
        }],
        remappings=remappings,
    )

    marker_spawner = Node(
        package='cartesian_interface_ros',
        executable='marker_spawner',
        name='interactive_markers',
        output='screen',
        condition=IfCondition(LaunchConfiguration('markers')),
        parameters=[{'tf_prefix': tf_prefix}],
    )

    return [cartesio_server, marker_spawner]


def generate_launch_description():
    pkg_share = FindPackageShare('concert_cartesio')
    default_ik = PathJoinSubstitution([pkg_share, 'concert_isaac_stack.yaml'])

    args = [
        DeclareLaunchArgument('ik', default_value=default_ik),
        DeclareLaunchArgument('rate', default_value='100.0'),
        DeclareLaunchArgument('tf_prefix', default_value='ci'),
        DeclareLaunchArgument('markers', default_value='true'),
        DeclareLaunchArgument('enable_otg', default_value='true'),
        DeclareLaunchArgument('is_floating_base', default_value='true'),
        DeclareLaunchArgument('model_type', default_value='pin'),
        DeclareLaunchArgument('solver', default_value='OpenSot'),
        # When false the node subscribes to the standard /robot_description
        # topic. When true it subscribes to /xbotcore/robot_description (the
        # topic that xbot2's ros2_io plugin republishes). The xbot2 path is
        # the production setup; the non-xbot path is useful for smoke tests
        # against a robot_state_publisher or an Isaac-side publisher.
        DeclareLaunchArgument('use_xbot_topics', default_value='true'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_build_nodes)])
