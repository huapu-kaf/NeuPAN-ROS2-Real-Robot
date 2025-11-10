#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Get the example configuration file
    package_dir = get_package_share_directory('neupan_ros')
    config_file = os.path.join(package_dir, 'example', 'gazebo_limo', 'dune_train_limo_cpu.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the neupan configuration file'
    )
    
    # Launch the neupan node
    neupan_node = Node(
        package='neupan_ros',
        executable='neupan_node',
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
    
    return LaunchDescription([
        config_file_arg,
        neupan_node
    ])