#!/usr/bin/env python3

"""
真实的NeuPAN导航测试 - 完整的导航栈测试
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Package directories
    neupan_ros_dir = get_package_share_directory('neupan_ros')
    
    # Configuration files
    rviz_config = os.path.join(neupan_ros_dir, 'rviz', 'simple_visual_test.rviz')
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='启动RViz进行可视化'
    )
    
    # 启动信息
    launch_info = ExecuteProcess(
        cmd=['echo', '🚀 启动真实的NeuPAN导航测试'],
        output='screen'
    )
    
    # 1. Static transforms
    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    
    static_tf_base_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # 2. Enhanced robot simulator
    robot_simulator = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/enhanced_robot_simulator',
        name='enhanced_robot_simulator',
        output='screen',
        parameters=[{
            'publish_rate': 20.0,
            'robot_radius': 0.161,
            'laser_range': 10.0,
            'laser_angle_min': -3.14,
            'laser_angle_max': 3.14,
            'laser_angle_increment': 0.0087,
            'environment_type': 'complex',
            'enable_obstacles': True,
        }]
    )
    
    # 3. Path publisher
    path_publisher = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/simple_path_publisher',
        name='path_publisher',
        output='screen',
        parameters=[{
            'waypoints': [
                0.0, 0.0, 0.0,
                2.0, 0.0, 0.0,
                4.0, 0.0, 1.57,
                4.0, 2.0, 3.14,
                2.0, 2.0, -1.57,
                0.0, 0.0, 0.0
            ]
        }]
    )
    
    # 4. 真实的NeuPAN控制器
    neupan_controller = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/real_neupan_controller',
        name='real_neupan_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # 5. Goal publisher
    goal_publisher = Node(
        package='neupan_ros',
        executable='/home/robotmaster/ros2_ws/install/neupan_ros/bin/goal_publisher_node',
        name='goal_publisher_node',
        output='screen',
        parameters=[{
            'goal_x': 4.0,
            'goal_y': 2.0,
            'goal_theta': 0.0
        }]
    )
    
    # 6. RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    
    return LaunchDescription([
        use_rviz_arg,
        
        # 启动信息
        launch_info,
        
        # 基础TF
        static_tf_odom_base,
        static_tf_base_laser,
        
        # 延时启动各个组件
        TimerAction(
            period=2.0,
            actions=[robot_simulator]
        ),
        
        TimerAction(
            period=4.0,
            actions=[path_publisher]
        ),
        
        # 关键: NeuPAN控制器
        TimerAction(
            period=6.0,
            actions=[neupan_controller]
        ),
        
        TimerAction(
            period=8.0,
            actions=[goal_publisher]
        ),
        
        # 可视化
        TimerAction(
            period=10.0,
            actions=[rviz_node]
        ),
    ])