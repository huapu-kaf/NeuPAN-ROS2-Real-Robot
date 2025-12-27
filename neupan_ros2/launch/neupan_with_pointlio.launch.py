#!/usr/bin/env python3

"""
NeuPAN + Point-LIO 集成Launch文件
用于实车导航，集成Point-LIO里程计
自动处理3D点云到2D LaserScan的转换 (使用本地pointcloud_to_laserscan源码包)

Usage:
    ros2 launch neupan_ros neupan_with_pointlio.launch.py config_file:=/path/to/config.yaml

Dependencies:
    - point_lio
    - pointcloud_to_laserscan (本地源码包)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='Path to NeuPAN config YAML file'
    )
    
    # 机器人相关参数 - Point-LIO默认Frame配置
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='camera_init',  # Point-LIO的全局坐标系
        description='Map frame (Point-LIO global frame)'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='aft_mapped',  # Point-LIO输出的机器人基座坐标系(TF child frame)
        description='Robot base frame (Point-LIO body frame in TF)'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame', 
        default_value='body',  # Point-LIO的点云坐标系
        description='Lidar frame (Point-LIO point cloud frame)'
    )
    
    # 话题配置
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/neupan/scan',  # 转换后的2D扫描话题
        description='Converted 2D Laser scan topic for NeuPAN'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command output topic'
    )
    
    # 3D点云输入话题 (来自Point-LIO)
    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/cloud_registered_body',
        description='Input point cloud topic from Point-LIO (must be in body frame)'
    )
    
    # PointCloud to LaserScan 节点
    # 使用本地pointcloud_to_laserscan源码包
    # 将Point-LIO的3D点云转换为NeuPAN需要的2D LaserScan
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', LaunchConfiguration('cloud_topic')),
            ('scan', LaunchConfiguration('scan_topic'))
        ],
        parameters=[{
            'target_frame': LaunchConfiguration('lidar_frame'), # 转换到body系
            'transform_tolerance': 0.01,
            'min_height': -1.0,  # 根据安装高度调整，这里假设取body周围所有点
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043, # 约0.25度
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True
        }],
        output='screen'
    )
    
    # NeuPAN主节点
    neupan_node = Node(
        package='neupan_ros',
        executable='neupan_node_ros2',
        name='neupan_planner',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'marker_size': 0.1,
            'marker_z': 0.0,
            'scan_range': '0.2 10.0', 
            'scan_angle_range': '-3.14159 3.14159',
            'scan_downsample': 2,
            'refresh_initial_path': True,
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
        cloud_topic_arg,
        
        pointcloud_to_laserscan_node,
        neupan_node
    ])
