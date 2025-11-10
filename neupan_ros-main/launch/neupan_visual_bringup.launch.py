#!/usr/bin/env python3

"""
NeuPAN visual bringup launch: starts simulator, path, real NeuPAN controller, and RViz.
Run:
  ros2 launch neupan_ros neupan_visual_bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('neupan_ros')
    rviz_config = os.path.join(pkg_share, 'rviz', 'basic_neupan.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Whether to run RViz'
    )

    banner = ExecuteProcess(
        cmd=['bash', '-lc', "echo '🚀 NeuPAN 可视化测试启动(模拟器+路径+控制器+RViz)'"],
        output='screen'
    )

    # Static TFs
    tf_odom_base = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )
    tf_base_laser = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    # Enhanced robot simulator (simple environment, no obstacles)
    robot_sim = ExecuteProcess(
        cmd=[
            '/home/robotmaster/ros2_ws/install/neupan_ros/bin/enhanced_robot_simulator',
            '--ros-args',
            '-p', 'publish_rate:=20.0',
            '-p', 'environment_type:=simple',
            '-p', 'enable_obstacles:=false',
            '-p', 'laser_range:=8.0',
            '-p', 'robot_radius:=0.2'
        ],
        output='screen'
    )

    # Path publisher (rectangle path)
    path_pub = ExecuteProcess(
        cmd=[
            '/home/robotmaster/ros2_ws/install/neupan_ros/bin/simple_path_publisher',
            '--ros-args',
            '-p', 'path_points:=[0.0,0.0, 3.0,0.0, 3.0,3.0, 0.0,3.0]',
            '-p', 'publish_rate:=2.0'
        ],
        output='screen'
    )

    # Real NeuPAN controller
    neupan_ctrl = ExecuteProcess(
        cmd=['/home/robotmaster/ros2_ws/install/neupan_ros/bin/real_neupan_controller'],
        additional_env={'PYTHONPATH': '/home/robotmaster/ros2_ws/src/NeuPAN'},
        output='screen'
    )

    # RViz2
    rviz = ExecuteProcess(
        condition=IfCondition(use_rviz),
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        use_rviz_arg,
        banner,
        tf_odom_base,
        tf_base_laser,
        TimerAction(period=2.0, actions=[robot_sim]),
        TimerAction(period=4.0, actions=[path_pub]),
        TimerAction(period=6.0, actions=[neupan_ctrl]),
        TimerAction(period=8.0, actions=[rviz]),
    ])
