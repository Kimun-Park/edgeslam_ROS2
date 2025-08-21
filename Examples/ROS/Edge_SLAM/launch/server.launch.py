#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('edge_slam_ros2')
    
    # Default paths - use absolute paths
    default_voc_path = '/home/isaacusr/edgeslam_ROS2/Vocabulary/ORBvoc.txt'
    default_settings_path = '/home/isaacusr/edgeslam_ROS2/Examples/RGB-D/carter.yaml'
    
    # Launch arguments
    voc_path_arg = DeclareLaunchArgument(
        'voc_path',
        default_value=default_voc_path,
        description='Path to ORB vocabulary file'
    )
    
    settings_path_arg = DeclareLaunchArgument(
        'settings_path',
        default_value=default_settings_path,
        description='Path to camera settings file'
    )
    
    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='127.0.0.1',
        description='Edge server IP address'
    )
    
    base_port_arg = DeclareLaunchArgument(
        'base_port',
        default_value='15000',
        description='Base port for TCP connections'
    )
    
    # Environment variables
    server_ip_env = SetEnvironmentVariable(
        'EDGE_SLAM_SERVER_IP',
        LaunchConfiguration('server_ip')
    )
    
    base_port_env = SetEnvironmentVariable(
        'EDGE_SLAM_BASE_PORT',
        LaunchConfiguration('base_port')
    )
    
    # Set robot_id to 0 for server (doesn't affect port allocation)
    robot_id_env = SetEnvironmentVariable(
        'EDGE_SLAM_ROBOT_ID',
        '0'
    )
    
    # Server node
    server_node = Node(
        package='edge_slam_ros2',
        executable='RGBD',
        name='edge_slam_server',
        arguments=[
            LaunchConfiguration('voc_path'),
            LaunchConfiguration('settings_path'),
            'server'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        voc_path_arg,
        settings_path_arg,
        server_ip_arg,
        base_port_arg,
        server_ip_env,
        base_port_env,
        robot_id_env,
        server_node
    ])