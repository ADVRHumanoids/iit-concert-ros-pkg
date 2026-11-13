import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('concert_mujoco')

    args = [
        DeclareLaunchArgument('xbot2', default_value='true'),
        DeclareLaunchArgument('xbot2_gui', default_value='true'),
        DeclareLaunchArgument('xbot2_config', default_value=os.path.join(share, 'config', 'modular.yaml')),
        DeclareLaunchArgument('modular_description', default_value=os.path.join(get_package_share_directory('concert_examples'), 'concert_example.py')),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'world', 'world.xml')),
        DeclareLaunchArgument('spawn_z', default_value='1.0'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('velodyne', default_value='false'),
        DeclareLaunchArgument('realsense', default_value='false'),
        DeclareLaunchArgument('ultrasound', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]

    # Link of each sensor in the URDF: a sensor is enabled only if requested and present
    sensor_links = {'imu': 'vectornav_imu_link', 'velodyne': 'VLP16_lidar_front',
                    'realsense': 'D435_camera_front_depth_frame', 'ultrasound': 'ultrasound_fl_sag'}

    # The robot model comes from the same modular generator as Gazebo, run once here.
    # xbot2_mujoco converts the URDF to MJCF and merges into it concert_mujoco.xml.xacro
    # (options, sensors) and robot_mjcf.py's output (actuators, joint classes, initial pose).
    def simulator(context):
        requested = ''.join(f' {s}:=' + LaunchConfiguration(s).perform(context) for s in sensor_links)
        generator = ['python3', LaunchConfiguration('modular_description').perform(context)]
        out_dir = tempfile.mkdtemp(prefix='concert_mujoco_')
        descriptions = {}
        for kind, xacro_args in (('urdf', 'gazebo_urdf:=false floating_base:=true'),
                                 ('srdf', 'gazebo_urdf:=false')):
            path = os.path.join(out_dir, f'modularbot.{kind}')
            result = subprocess.run(generator + ['-o', kind, '-a'] + (xacro_args + requested).split()
                                    + ['-r', 'modularbot'], capture_output=True, text=True)
            if result.returncode != 0:
                raise RuntimeError(f'{kind} generation failed:\n{result.stderr}')
            with open(path, 'w') as f:
                f.write(result.stdout)
            descriptions[kind] = path

        links = {link.get('name') for link in ET.parse(descriptions['urdf']).getroot().iter('link')}
        enabled = {s: LaunchConfiguration(s).perform(context) == 'true' and link in links
                   for s, link in sensor_links.items()}
        robot_mjcf = os.path.join(get_package_prefix('concert_mujoco'), 'lib', 'concert_mujoco', 'robot_mjcf.py')

        cmd = [
            'python3', '-m', 'xbot2_py_mujoco.simulate',
            '--name', 'concert',
            '--urdf', descriptions['urdf'],
            '--srdf', descriptions['srdf'],
            '--xml-cmd', 'xacro ' + os.path.join(share, 'mjcf', 'concert_mujoco.xml.xacro')
                         + ' spawn_z:=' + LaunchConfiguration('spawn_z').perform(context)
                         + ''.join(f' {s}:={str(e).lower()}' for s, e in enabled.items()),
            '--xml-cmd', f'python3 {robot_mjcf} {descriptions["urdf"]} {descriptions["srdf"]}',
            '--xml', LaunchConfiguration('world').perform(context),
        ]
        if LaunchConfiguration('headless').perform(context) == 'true':
            cmd.append('--headless')
        actions = [ExecuteProcess(cmd=cmd, additional_env={'PYTHONUNBUFFERED': '1'}, output='screen')]

        # The gz depth sensors also publish a cloud on <depth topic>/points
        if enabled['realsense']:
            for camera in ('D435_camera_front', 'D435_camera_back'):
                actions.append(Node(
                    package='depth_image_proc', executable='point_cloud_xyz_node',
                    name=camera + '_points',
                    remappings=[('image_rect', f'/{camera}/depth/image_raw'),
                                ('camera_info', f'/{camera}/depth/camera_info'),
                                ('points', f'/{camera}/depth/image_raw/points')],
                    parameters=[{'use_sim_time': True}],
                    output='log',
                ))
        return actions

    # The bridge socket exists once the simulator publishes /clock.
    # --simtime MUST NOT be passed: it waits for Gazebo's time socket.
    xbot2_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2')),
        cmd=[
            'bash', '-c',
            ['ros2 topic echo --once /clock rosgraph_msgs/msg/Clock > /dev/null && exec xbot2-core -V --hw mj --config ',
             LaunchConfiguration('xbot2_config'), ' --'],
        ],
        output='screen',
    )

    xbot2_gui_server = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2_gui')),
        cmd=['xbot2_gui_server', os.path.join(get_package_share_directory('concert_gazebo'), 'config', 'gui', 'gui_server_config.yaml')],
        output='log',
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('concert_gazebo'), 'rviz', 'concert_sensors.rviz')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [OpaqueFunction(function=simulator), xbot2_process, xbot2_gui_server, rviz_node])
