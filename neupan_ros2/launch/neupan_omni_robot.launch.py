#!/usr/bin/env python3

"""
Launch file for NeuPAN omnidirectional robot navigation with Point-LIO integration.

This launch file starts:
- NeuPAN navigation node configured for omnidirectional robot
- RViz2 for visualization
- Point-LIO integration for localization (optional)

Author: Ruihua Han
Date: 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='example/non_obs/omni/planner.yaml',
        description='Path to the NeuPAN omnidirectional robot configuration file'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    use_pointlio_arg = DeclareLaunchArgument(
        'use_pointlio',
        default_value='false',
        description='Whether to start Point-LIO for localization'
    )
    
    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='camera_init',
        description='Map frame (camera_init for Point-LIO, map for other SLAM)'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='velodyne',
        description='LiDAR frame'
    )

    # Get package share directory
    neupan_ros_share = get_package_share_directory('neupan_ros')
    
    # NeuPAN omnidirectional robot node
    neupan_node = Node(
        package='neupan_ros',
        executable='neupan_node',
        name='neupan_omni_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'marker_size': 0.05,
            'marker_z': 1.0,
            'scan_angle_range': '-3.14 3.14',
            'scan_downsample': 1,
            'scan_range': '0.0 5.0',
            'refresh_initial_path': False,
            'flip_angle': False,
            'include_initial_path_direction': False
        }],
        remappings=[
            ('/scan', '/livox/lidar'),  # For Point-LIO integration
            ('/neupan_cmd_vel', '/cmd_vel')
        ]
    )
    
    # RViz2 configuration for omnidirectional robot
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('neupan_ros'),
        'rviz',
        'neupan_omni_navigation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    # Point-LIO launch (optional)
    pointlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('point_lio'),
                'launch',
                'mapping.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_pointlio'))
    )

    return LaunchDescription([
        config_file_arg,
        use_rviz_arg,
        use_pointlio_arg,
        map_frame_arg,
        base_frame_arg,
        lidar_frame_arg,
        neupan_node,
        rviz_node,
        pointlio_launch
    ]) 