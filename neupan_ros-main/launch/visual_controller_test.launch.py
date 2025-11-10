#!/usr/bin/env python3

"""
简化的NeuPAN控制器可视化测试
专注于可视化效果，避免复杂的初始化问题
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Package directories
    neupan_ros_dir = get_package_share_directory('neupan_ros')
    
    # Configuration files
    minimal_config = os.path.join(neupan_ros_dir, '..', '..', 'src', 'neupan_ros', 'config', 'minimal_test_config.yaml')
    rviz_config = os.path.join(neupan_ros_dir, 'rviz', 'simple_visual_test.rviz')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动RViz进行可视化'
    )
    
    # 1. Static transform publisher (base_link -> odom)
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    # 2. Enhanced robot simulator (provides odometry and laser scan)
    robot_simulator = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/enhanced_robot_simulator',
        name='enhanced_robot_simulator',
        output='screen',
        parameters=[{
            'publish_rate': 20.0,
            'robot_radius': 0.2,
            'laser_range': 5.0,
            'laser_angle_min': -3.14,
            'laser_angle_max': 3.14,
            'laser_angle_increment': 0.0174,  # 1 degree
            'environment_type': 'simple',  # 简单环境避免复杂计算
        }]
    )
    
    # 3. Simple path publisher
    path_publisher = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/simple_path_publisher',
        name='simple_path_publisher',
        output='screen',
        parameters=[{
            'waypoints': [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 4.0, 2.0, 0.0, 2.0, 4.0, 1.57]
        }]
    )
    
    # 4. Manual velocity publisher for testing (模拟控制器输出)
    manual_velocity_node = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/manual_velocity_publisher',
        name='manual_velocity_publisher',
        output='screen'
    )
    
    # 5. Goal publisher for visualization
    goal_publisher = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/goal_publisher_node',
        name='goal_publisher_node',
        output='screen',
        parameters=[{
            'goal_x': 4.0,
            'goal_y': 2.0,
            'goal_theta': 1.57
        }]
    )
    
    # 6. RViz for visualization  
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    # 添加启动信息
    launch_info = ExecuteProcess(
        cmd=['echo', '🚀 启动NeuPAN控制器可视化测试\n📊 检查RViz中的以下内容:\n  - 机器人位置 (base_link)\n  - 激光雷达数据 (LaserScan)\n  - 路径规划 (Path)\n  - 目标点 (Goal)\n  - 速度命令 (cmd_vel可通过rostopic echo查看)'],
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        
        # 启动信息
        launch_info,
        
        # 核心节点
        static_transform_publisher,
        
        TimerAction(
            period=2.0,
            actions=[robot_simulator]
        ),
        
        TimerAction(
            period=3.0,
            actions=[path_publisher]
        ),
        
        TimerAction(
            period=4.0,
            actions=[manual_velocity_node]
        ),
        
        TimerAction(
            period=5.0,
            actions=[goal_publisher]
        ),
        
        # 可视化
        TimerAction(
            period=6.0,
            actions=[rviz_node]
        ),
    ])