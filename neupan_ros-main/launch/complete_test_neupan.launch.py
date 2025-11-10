#!/usr/bin/env python3
"""
完整的NeuPAN控制器测试 - 包含仿真器、控制器和可视化
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
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
    
    # 1. 启动增强机器人仿真器 (提供里程计、激光扫描和TF)
    robot_simulator = Node(
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/enhanced_robot_simulator',
        name='robot_simulator',
        parameters=[
            {'use_sim_time': False},
            {'environment_type': 'simple'},
            {'num_static_obstacles': 3},
            {'num_dynamic_obstacles': 1},
            {'world_size_x': 4.0},
            {'world_size_y': 4.0},
            {'laser_range': 3.0}
        ],
        output='screen'
    )
    
    # 2. TF静态变换发布器
    laser_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=['0.1', '0', '0.1', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )
    
    # 3. 简单的路径发布器 (提供测试路径)
    path_publisher = Node(
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/simple_path_publisher',
        name='path_publisher',
        parameters=[
            {'use_sim_time': False},
            {'path_points': [0.0, 0.0, 1.5, 0.0, 1.5, 1.5, 0.0, 1.5]},
            {'frame_id': 'odom'},
            {'publish_rate': 2.0}
        ],
        output='screen'
    )
    
    # 4. 启动Nav2控制器服务器 (使用NeuPAN控制器)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[PathJoinSubstitution([neupan_pkg_dir, 'config', 'test_neupan_controller.yaml'])],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 5. 生命周期管理器 (用于激活控制器)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['controller_server']
        }],
        output='screen'
    )
    
    # 6. 简单的导航目标发布器
    navigation_test = Node(
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/goal_publisher_node',
        name='navigation_test',
        parameters=[
            {'use_sim_time': False},
            {'goal_x': 1.5},
            {'goal_y': 1.5},
            {'auto_publish': True},
            {'goal_interval': 15.0}
        ],
        output='screen'
    )
    
    # 7. RViz2可视化
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
        
        # Components - 按启动顺序
        robot_simulator,
        laser_tf_publisher,
        
        # 延时启动控制器相关组件
        TimerAction(
            period=2.0,
            actions=[path_publisher]
        ),
        TimerAction(
            period=3.0,
            actions=[controller_server]
        ),
        TimerAction(
            period=4.0,
            actions=[lifecycle_manager]
        ),
        TimerAction(
            period=6.0,
            actions=[navigation_test]
        ),
        TimerAction(
            period=1.0,
            actions=[rviz_node]
        ),
    ])