#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Get the example configuration file
    package_dir = get_package_share_directory('neupan_ros')
    config_file = os.path.join(package_dir, 'example', 'gazebo_limo', 'dune_train_limo_cpu.yaml')
    rviz_config_file = os.path.join(package_dir, 'rviz', 'neupan_visualization.rviz')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the neupan configuration file'
    )
    
    # Robot simulator node
    robot_simulator = Node(
        executable=os.path.join(get_package_share_directory('neupan_ros'), '..', '..', 'bin', 'simple_robot_simulator'),
        name='simple_robot_simulator',
        output='screen'
    )
    
    # NeuPAN algorithm node
    neupan_node = Node(
        executable=os.path.join(get_package_share_directory('neupan_ros'), '..', '..', 'bin', 'neupan_node'),
        name='neupan_node',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'map_frame': 'map',
            'base_frame': 'base_link',
            'lidar_frame': 'laser_link',
            'marker_size': 0.05,
            'marker_z': 1.0,
            'scan_angle_range': '-3.14 3.14',
            'scan_downsample': 1,
            'scan_range': '0.1 5.0',
            'refresh_initial_path': False,
            'flip_angle': False,
            'include_initial_path_direction': False
        }],
        output='screen'
    )
    
    # RViz2 node
    rviz_node = TimerAction(
        period=2.0,  # Wait 2 seconds before starting RViz
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                output='screen'
            )
        ]
    )
    
    # Goal publisher node (delayed start)
    goal_publisher = TimerAction(
        period=5.0,  # Wait 5 seconds before publishing goal
        actions=[
            Node(
                executable=os.path.join(get_package_share_directory('neupan_ros'), '..', '..', 'bin', 'goal_publisher'),
                name='goal_publisher',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        robot_simulator,
        neupan_node,
        rviz_node,
        goal_publisher
    ])