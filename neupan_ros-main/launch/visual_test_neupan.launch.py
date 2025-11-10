#!/usr/bin/env python3
"""
NeuPAN控制器可视化测试launch文件
只启动基础的仿真组件和RViz进行可视化测试
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    neupan_pkg_dir = FindPackageShare('neupan_ros')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # 启动增强机器人仿真器 (提供里程计、激光扫描和TF)
    robot_simulator = Node(
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/enhanced_robot_simulator',
        name='robot_simulator',
        parameters=[
            {'use_sim_time': False},
            {'environment_type': 'simple'},  # 简单环境便于测试
            {'num_static_obstacles': 3},
            {'num_dynamic_obstacles': 1},
            {'world_size_x': 4.0},
            {'world_size_y': 4.0},
            {'laser_range': 3.0}
        ],
        output='screen'
    )
    
    # 发布laser_link到base_link的静态TF变换
    laser_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )
    
    # 简单的路径发布器 (提供测试路径)
    path_publisher = Node(
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/simple_path_publisher',
        name='path_publisher',
        parameters=[
            {'use_sim_time': False},
            {'path_points': [0.0, 0.0, 1.5, 0.0, 1.5, 1.5, 0.0, 1.5]},
            {'frame_id': 'odom'},
            {'publish_rate': 1.0}
        ],
        output='screen'
    )
    
    # RViz2用于可视化
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([neupan_pkg_dir, 'rviz', 'neupan_controller_test.rviz'])],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        
        # Components
        robot_simulator,
        laser_tf_publisher,
        TimerAction(
            period=2.0,
            actions=[path_publisher]
        ),
        TimerAction(
            period=1.0,
            actions=[rviz_node]
        ),
    ])