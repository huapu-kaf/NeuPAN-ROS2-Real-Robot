#!/usr/bin/env python3
"""
NeuPAN控制器独立测试launch文件
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
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # 启动增强机器人仿真器 (提供里程计和激光扫描)
    robot_simulator = Node(
        package='neupan_ros',
        executable='enhanced_robot_simulator',
        name='robot_simulator',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'environment_type': 'simple'},  # 简单环境便于测试
            {'num_static_obstacles': 5},
            {'num_dynamic_obstacles': 1},
            {'world_size_x': 6.0},
            {'world_size_y': 6.0},
            {'laser_range': 5.0}
        ],
        output='screen'
    )
    
    # 启动Nav2控制器服务器 (仅控制器，不包含其他Nav2组件)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[PathJoinSubstitution([neupan_pkg_dir, 'config', 'test_neupan_controller.yaml'])],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 简单的路径发布器 (提供全局路径给控制器)
    path_publisher = Node(
        package='neupan_ros',
        executable='simple_path_publisher',
        name='path_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'path_points': [0.0, 0.0, 1.0, 0.0, 2.0, 1.0, 3.0, 2.0]}
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_rviz_arg,
        
        # Components
        robot_simulator,
        TimerAction(
            period=2.0,
            actions=[controller_server]
        ),
        TimerAction(
            period=4.0,
            actions=[path_publisher]
        ),
        TimerAction(
            period=3.0,
            actions=[rviz_node]
        ),
    ])