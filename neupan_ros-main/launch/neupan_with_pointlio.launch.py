#!/usr/bin/env python3

"""
NeuPAN + Point-LIO 集成Launch文件
用于实车导航，集成Point-LIO里程计

Usage:
    ros2 launch neupan_ros neupan_with_pointlio.launch.py config_file:=/path/to/config.yaml

需要预先启动:
    1. 激光雷达驱动
    2. Point-LIO节点
    3. 机器人底盘驱动
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # 获取包路径
    pkg_share = get_package_share_directory('neupan_ros')
    
    # Launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        description='Path to NeuPAN config YAML file'
    )
    
    # 机器人相关参数
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='Robot base frame'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame', 
        default_value='velodyne',  # Point-LIO通常使用velodyne坐标系
        description='Lidar frame (for Point-LIO integration)'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='camera_init',  # Point-LIO的地图坐标系
        description='Map frame (Point-LIO map frame)'
    )
    
    # 话题重映射参数
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/livox/lidar',  # Livox激光雷达话题
        description='Laser scan topic'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Velocity command output topic'
    )
    
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/Odometry',  # Point-LIO里程计话题
        description='Odometry topic from Point-LIO'
    )
    
    # NeuPAN配置参数
    safety_margin_arg = DeclareLaunchArgument(
        'safety_margin',
        default_value='0.3',
        description='Safety margin for obstacle avoidance (meters)'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='1.0',
        description='Maximum robot speed (m/s)'
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
            'base_frame': LaunchConfiguration('robot_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'marker_size': 0.1,  # 实车中可以稍大一些便于观察
            'marker_z': 0.5,
            'scan_range': '0.5 8.0',  # 适合室外环境的扫描范围
            'scan_angle_range': '-3.14159 3.14159',
            'scan_downsample': 2,  # 减少计算负载
            'refresh_initial_path': True,
            'flip_angle': False,
            'include_initial_path_direction': False
        }],
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')),
            ('/neupan_cmd_vel', LaunchConfiguration('cmd_vel_topic'))
        ]
    )
    
    # 安全监控节点 (可选)
    safety_monitor = Node(
        package='neupan_ros',
        executable='safety_monitor.py',  # 需要单独实现
        name='safety_monitor',
        output='screen',
        parameters=[{
            'emergency_stop_distance': 0.5,  # 紧急停止距离
            'max_cmd_vel_age': 1.0,  # 控制命令最大延迟
        }],
        condition='false'  # 默认禁用，可根据需要启用
    )
    
    # TF发布节点 - 发布静态变换 (如果需要)
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='neupan_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 
                   LaunchConfiguration('robot_frame'), 
                   LaunchConfiguration('lidar_frame')],
        condition='false'  # 默认禁用，因为Point-LIO通常会发布相关TF
    )
    
    return LaunchDescription([
        # 参数声明
        config_file_arg,
        robot_frame_arg,
        lidar_frame_arg,
        map_frame_arg,
        scan_topic_arg,
        cmd_vel_topic_arg,
        odom_topic_arg,
        safety_margin_arg,
        max_speed_arg,
        
        # 节点启动
        neupan_node,
        # safety_monitor,  # 可选
        # static_tf_publisher,  # 可选
    ]) 