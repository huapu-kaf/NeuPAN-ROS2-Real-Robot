#!/usr/bin/env python3

"""
NeuPAN ROS2 Launch File
适用于ROS2 Humble

Usage:
    ros2 launch neupan_ros neupan_ros2.launch.py

Parameters:
    config_file: NeuPAN配置文件路径 (必需)
    map_frame: 地图坐标系 (default: map)
    base_frame: 机器人基础坐标系 (default: base_link)  
    lidar_frame: 激光雷达坐标系 (default: laser_link)
    scan_topic: 激光雷达话题 (default: /scan)
    cmd_vel_topic: 速度控制话题 (default: /cmd_vel)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # 获取包的安装路径
    pkg_share = get_package_share_directory('neupan_ros')
    
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='Path to NeuPAN config YAML file'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame', 
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser_link', 
        description='Lidar frame ID'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command topic'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='Visualization marker size'
    )
    
    scan_range_arg = DeclareLaunchArgument(
        'scan_range',
        default_value='0.0 5.0',
        description='Valid scan range [min, max] in meters'
    )
    
    scan_angle_range_arg = DeclareLaunchArgument(
        'scan_angle_range', 
        default_value='-3.14 3.14',
        description='Valid scan angle range [min, max] in radians'
    )
    
    # NeuPAN节点
    neupan_node = Node(
        package='neupan_ros',
        executable='neupan_node_ros2',
        name='neupan_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'marker_size': LaunchConfiguration('marker_size'),
            'scan_range': LaunchConfiguration('scan_range'),
            'scan_angle_range': LaunchConfiguration('scan_angle_range'),
            'scan_downsample': 1,
            'refresh_initial_path': False,
            'flip_angle': False,
            'include_initial_path_direction': False
        }],
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')),
            ('/neupan_cmd_vel', LaunchConfiguration('cmd_vel_topic'))
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        map_frame_arg,
        base_frame_arg,
        lidar_frame_arg,
        scan_topic_arg,
        cmd_vel_topic_arg,
        marker_size_arg,
        scan_range_arg,
        scan_angle_range_arg,
        neupan_node
    ]) 