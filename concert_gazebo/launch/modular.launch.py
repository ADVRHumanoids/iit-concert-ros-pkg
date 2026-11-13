from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    arg_launch_arguments = [
        DeclareLaunchArgument('gazebo', default_value='true'),
        DeclareLaunchArgument('xbot2', default_value='true'),
        DeclareLaunchArgument('xbot2_gui', default_value='true'),
        DeclareLaunchArgument('xbot2_config', default_value=os.path.join(get_package_share_directory('concert_gazebo'), 'config', 'modular.yaml')),
        DeclareLaunchArgument('modular_description', default_value=os.path.join(get_package_share_directory('concert_examples'), 'concert_example.py')),
        DeclareLaunchArgument('realsense', default_value='false'),
        DeclareLaunchArgument('velodyne', default_value='false'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('ultrasound', default_value='false'),
        DeclareLaunchArgument('use_gpu_ray', default_value='false'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('physics', default_value='ode'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('extra_gazebo_args', default_value=''),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(get_package_share_directory('concert_gazebo'), 'world/empty_world.sdf'))
    ]


    # Construct `gz_args`: -v if verbose, -s if no gui, -r unless paused
    gz_args = [
        LaunchConfiguration('world_file'),
        PythonExpression(["' -v' if '", LaunchConfiguration('verbose'), "' == 'true' else ''"]),
        PythonExpression(["' -s' if '", LaunchConfiguration('gui'), "' == 'false' else ''"]),
        PythonExpression(["'' if '", LaunchConfiguration('paused'), "' == 'true' else ' -r'"]),
    ]

    # Robot description commands
    robot_description_gz = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf -a gazebo_urdf:=true floating_base:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' imu:=', LaunchConfiguration('imu'),
        ' ultrasound:=', LaunchConfiguration('ultrasound'),
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot_gz'
    ],
    on_stderr='ignore'
    )

    robot_description_xbot = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o urdf -a gazebo_urdf:=false floating_base:=true',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' imu:=', LaunchConfiguration('imu'),
        ' ultrasound:=false',
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot'
    ],
    on_stderr='ignore'
    )

    robot_description_semantic = Command([
        'python3', ' ', LaunchConfiguration('modular_description'),
        ' -o srdf -a gazebo_urdf:=false',
        ' realsense:=', LaunchConfiguration('realsense'),
        ' velodyne:=', LaunchConfiguration('velodyne'),
        ' imu:=', LaunchConfiguration('imu'),
        ' ultrasound:=false',
        ' use_gpu_ray:=', LaunchConfiguration('use_gpu_ray'),
        ' -r modularbot'
    ],
    on_stderr='ignore'
    )

    # Robot description publisher node
    description_publisher_node = Node(
        package='xbot2_ros',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        parameters=[
            {'robot_description': ParameterValue(robot_description_xbot, value_type=str)},
            {'robot_description_semantic': ParameterValue(robot_description_semantic, value_type=str)}
        ],
        output='screen'
    )

    # Gazebo group
    gazebo_group = GroupAction(condition=IfCondition(LaunchConfiguration('gazebo')), actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='urdf_spawner',
            parameters=[{'string': ParameterValue(robot_description_gz, value_type=str), 'z': 1.0}]
        ),
        # Bridge for standard topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                # Clock
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # IMU, on the gz topic of the vectornav_imu sensor
                '/vectornav_imu_link@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Velodyne LiDARs
                '/VLP16_lidar_back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/VLP16_lidar_front/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[('/vectornav_imu_link', '/vectornav/imu')],
        ),
        # RealSense D435 colour images (realsense_gazebo_description topics)
        *[Node(
            package='ros_gz_image',
            executable='image_bridge',
            name=f'{camera}_color_bridge',
            arguments=[f'/{camera}/color/image_raw'],
        ) for camera in ('D435_camera_front', 'D435_camera_back')],
        # RealSense D435 depth, camera infos and clouds; Bosch USS5 ultrasound scans and clouds
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_sensors_bridge',
            arguments=[
                topic
                for camera in ('D435_camera_front', 'D435_camera_back')
                for topic in (
                    f'/{camera}/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera}/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{camera}/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{camera}/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                )
            ] + [
                topic
                for name in ('fl_sag', 'fr_sag', 'rl_sag', 'rr_sag', 'fl_lat', 'fr_lat', 'rl_lat', 'rr_lat')
                for topic in (
                    f'/bosch_uss5/ultrasound_{name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/bosch_uss5/ultrasound_{name}/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                )
            ],
        ),
    ])

    # Xbot2 process
    xbot2_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2')),
        cmd=[
            'xbot2-core', '-V', '--hw', 'sim', '--simtime',
            '--config', LaunchConfiguration('xbot2_config'), '--'
        ],
        output='screen'
    )

    # Xbot2 GUI server
    xbot2_gui_server = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('xbot2_gui')),
        cmd=['xbot2_gui_server', os.path.join(get_package_share_directory('concert_gazebo'), 'config', 'gui', 'gui_server_config.yaml')],
        output='log'
    )

    # xbot2_gui_client = ExecuteProcess(
    #     condition=IfCondition(LaunchConfiguration('xbot2_gui')),
    #     cmd=['xbot2_gui'],
    #     output='log'
    # )

    # RViz node
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('concert_gazebo'), 'rviz/concert_sensors.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Create and return launch description
    return LaunchDescription(arg_launch_arguments + [
        description_publisher_node,
        gazebo_group,
        xbot2_process,
        xbot2_gui_server,
#        xbot2_gui_client,
        rviz_node
    ])