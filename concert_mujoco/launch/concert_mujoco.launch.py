import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


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
        DeclareLaunchArgument('headless', default_value='false'),
    ]

    # The robot model comes from the same modular generator as Gazebo;
    # xbot2_mujoco converts the URDF to MJCF and merges concert_mujoco.xml.xacro into it.
    def simulator(context):
        imu = ' imu:=' + LaunchConfiguration('imu').perform(context)
        generator = 'python3 ' + LaunchConfiguration('modular_description').perform(context)
        cmd = [
            'python3', '-m', 'xbot2_py_mujoco.simulate',
            '--name', 'concert',
            '--urdf-cmd', generator + ' -o urdf -a gazebo_urdf:=false floating_base:=true' + imu + ' -r modularbot',
            '--srdf-cmd', generator + ' -o srdf -a gazebo_urdf:=false' + imu + ' -r modularbot',
            '--xml-cmd', 'xacro ' + os.path.join(share, 'mjcf', 'concert_mujoco.xml.xacro')
                         + ' spawn_z:=' + LaunchConfiguration('spawn_z').perform(context) + imu,
            '--xml', LaunchConfiguration('world').perform(context),
        ]
        if LaunchConfiguration('headless').perform(context) == 'true':
            cmd.append('--headless')
        return [ExecuteProcess(cmd=cmd, additional_env={'PYTHONUNBUFFERED': '1'}, output='screen')]

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

    return LaunchDescription(args + [OpaqueFunction(function=simulator), xbot2_process, xbot2_gui_server])
