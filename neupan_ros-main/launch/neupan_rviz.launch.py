#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Get the package directory
    package_dir = get_package_share_directory('neupan_ros')
    
    # RViz2 configuration file
    rviz_config_file = os.path.join(package_dir, 'rviz', 'neupan_visualization.rviz')
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_node
    ])