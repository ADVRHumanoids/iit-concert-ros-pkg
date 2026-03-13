#!/usr/bin/env python3
"""
Launch the Velodyne driver and pointcloud convert nodes with default configuration.

This launch file:
  - Loads parameters from YAML files for both the velodyne driver and convert nodes.
  - Overrides the 'calibration' parameter for the convert node with a local path 
    to the calibration file.
  - Shuts down the entire launch when the convert node exits.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description for Velodyne driver and convert nodes."""
    
    velodyne_dir = get_package_share_directory('velodyne_pointcloud')
    concert_config_dir = get_package_share_directory('concert_config')
    
    # Load convert node parameters
    convert_params_file = os.path.join(concert_config_dir, 'config','VLP16_front-velodyne_convert_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        all_params_convert = yaml.safe_load(f)
        params_convert = all_params_convert['velodyne_convert_node']['ros__parameters']
    
    # Override calibration file path
    calibration_file = os.path.join(velodyne_dir, 'params', 'VLP16db.yaml')
    params_convert['calibration'] = calibration_file

    # Velodyne convert node
    velodyne_convert_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        name='velodyne_convert_node',
        output='screen',
        parameters=[params_convert],
        namespace='VLP16_lidar_front',
    )

    # Load driver node parameters
    driver_params_file = os.path.join(concert_config_dir, 'config', 'VLP16_front-velodyne_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        all_params_driver = yaml.safe_load(f)
        params_driver = all_params_driver['velodyne_driver_node']['ros__parameters']
        
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        name='velodyne_driver_node',
        output='screen',
        parameters=[params_driver],
        namespace='VLP16_lidar_front',
    )

    # Shut down when the convert node exits
    exit_handler_convert_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velodyne_convert_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    
    # Shut down when the driver node exits
    exit_handler_driver_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velodyne_driver_node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    return LaunchDescription([
        velodyne_convert_node,
        velodyne_driver_node,
        exit_handler_convert_node,
        exit_handler_driver_node,
    ])
