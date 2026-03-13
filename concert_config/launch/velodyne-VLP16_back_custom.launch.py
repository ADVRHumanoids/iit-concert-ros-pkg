import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ###############################
    # Velodyne Transform Node Args
    ###############################
    
    calibration_arg = DeclareLaunchArgument(
        'calibration',
        default_value='',
        description=(
            'Path to the calibration file for the particular device. '
            'If empty, the default from the included launch file or parameter file is used.'
        )
    )
    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.9',
        description=(
            'The minimum range (meters) that a point must be to be included. '
            'Must be between 0.1 and 10.0.'
        )
    )
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='130.0',
        description=(
            'The maximum range (meters) that a point must be to be included. '
            'Must be between 0.1 and 200.0.'
        )
    )
    view_direction_arg = DeclareLaunchArgument(
        'view_direction',
        default_value='0.0',
        description=(
            'The center of the viewing angle around the circumference of the device, '
            'in radians (-Pi to Pi).'
        )
    )
    view_width_arg = DeclareLaunchArgument(
        'view_width',
        default_value='6.283185307',  # 2*Pi
        description=(
            'The width, in radians, of the arc to generate the pointcloud for '
            '(0 to 2Pi). Defaults to 2Pi.'
        )
    )
    organize_cloud_arg = DeclareLaunchArgument(
        'organize_cloud',
        default_value='True',
        description=(
            'Whether to organize the cloud by ring (True) or use the raw order (False).'
        )
    )
    target_frame_arg = DeclareLaunchArgument(
        'target_frame',
        default_value='',
        description=(
            'The coordinate frame to place in the header of the published pointcloud. '
            'If empty, the frame from the driver packets is used.'
        )
    )
    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='VLP16_lidar_back_base_link',
        description=(
            'The fixed coordinate frame to transform the data from. '
            'If empty, no additional transform is applied.'
        )
    )

    
    ############################
    # Velodyne Driver Node Args
    ############################
    device_ip_arg = DeclareLaunchArgument(
        'device_ip',
        default_value='10.24.10.202',
        description='IP address of the Velodyne.'
    )
    gps_time_arg = DeclareLaunchArgument(
        'gps_time',
        default_value='False',
        description='Use GPS time (True) or local time (False).'
    )
    time_offset_arg = DeclareLaunchArgument(
        'time_offset',
        default_value='0.0',
        description='Time offset (in seconds) added to the acquisition timestamp.'
    )
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='True',
        description='Whether the device starts enabled or not.'
    )
    read_once_arg = DeclareLaunchArgument(
        'read_once',
        default_value='False',
        description='Only playback the data once (True) or continuously (False), for PCAP mode.'
    )
    read_fast_arg = DeclareLaunchArgument(
        'read_fast',
        default_value='False',
        description='Output data as fast as possible (True) or in real-time (False), for PCAP mode.'
    )
    repeat_delay_arg = DeclareLaunchArgument(
        'repeat_delay',
        default_value='0.0',
        description='Time to wait between repeats in continuous PCAP playback mode.'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='VLP16_lidar_back_base_link',
        description='Frame ID used in the published packets.'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='VLP16',
        description='Velodyne model number (e.g. 64E, 32E, VLP16, etc.).'
    )
    rpm_arg = DeclareLaunchArgument(
        'rpm',
        default_value='600.0',
        description='Descriptive RPM that matches the Velodyne web interface.'
    )
    pcap_arg = DeclareLaunchArgument(
        'pcap',
        default_value='',
        description='Path to a PCAP file to playback data. Empty for live mode.'
    )
    cut_angle_arg = DeclareLaunchArgument(
        'cut_angle',
        default_value='-1.0',
        description='Azimuth angle at which a rotation is considered complete.'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='2369',
        description='Port to receive data from the Velodyne.'
    )
    
    ###############################
    # Pointcloud Launch
    ###############################
    # velodyne_pointcloud_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('velodyne_pointcloud'), 'launch', 'velodyne_transform_node-VLP16-launch.py')),
    #     launch_arguments={
    #         'calibration':    LaunchConfiguration('calibration'),
    #         'min_range':      LaunchConfiguration('min_range'),
    #         'max_range':      LaunchConfiguration('max_range'),
    #         'view_direction': LaunchConfiguration('view_direction'),
    #         'view_width':     LaunchConfiguration('view_width'),
    #         'organize_cloud': LaunchConfiguration('organize_cloud'),
    #         'target_frame':   LaunchConfiguration('target_frame'),
    #         'fixed_frame':    LaunchConfiguration('fixed_frame'),
    #     }.items()
    # )
    
    velodyne_pointcloud_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('concert_config'), 'launch', 'velodyne_convert_node-VLP16-launch.py')),
        launch_arguments={
            'calibration':    LaunchConfiguration('calibration'),
            'min_range':      LaunchConfiguration('min_range'),
            'max_range':      LaunchConfiguration('max_range'),
            'view_direction': LaunchConfiguration('view_direction'),
            'view_width':     LaunchConfiguration('view_width'),
            'organize_cloud': LaunchConfiguration('organize_cloud'),
            'target_frame':   LaunchConfiguration('target_frame'),
            'fixed_frame':    LaunchConfiguration('fixed_frame'),
        }.items()
    )
    
    
    
    ###############################
    # Driver Launch
    ###############################
    velodyne_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('velodyne_driver'), 'launch', 'velodyne_driver_node-VLP16-launch.py')),
        launch_arguments={
            # Driver node parameters
            'device_ip':    LaunchConfiguration('device_ip'),
            'gps_time':     LaunchConfiguration('gps_time'),
            'time_offset':  LaunchConfiguration('time_offset'),
            'enabled':      LaunchConfiguration('enabled'),
            'read_once':    LaunchConfiguration('read_once'),
            'read_fast':    LaunchConfiguration('read_fast'),
            'repeat_delay': LaunchConfiguration('repeat_delay'),
            'frame_id':     LaunchConfiguration('frame_id'),
            'model':        LaunchConfiguration('model'),
            'rpm':          LaunchConfiguration('rpm'),
            'pcap':         LaunchConfiguration('pcap'),
            'cut_angle':    LaunchConfiguration('cut_angle'),
            'port':         LaunchConfiguration('port'),
        }.items()
    )
    
    
    
    return LaunchDescription([
        # Transform node parameters
        calibration_arg,
        min_range_arg,
        max_range_arg,
        view_direction_arg,
        view_width_arg,
        organize_cloud_arg,
        target_frame_arg,
        fixed_frame_arg,

        # Driver node parameters
        device_ip_arg,
        gps_time_arg,
        time_offset_arg,
        enabled_arg,
        read_once_arg,
        read_fast_arg,
        repeat_delay_arg,
        frame_id_arg,
        model_arg,
        rpm_arg,
        pcap_arg,
        cut_angle_arg,
        port_arg,

        # Actions
        velodyne_pointcloud_launch,
        velodyne_driver_launch
    ])