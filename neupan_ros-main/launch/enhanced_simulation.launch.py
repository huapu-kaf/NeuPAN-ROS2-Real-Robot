#!/usr/bin/env python3
"""
Enhanced simulation launch file for NeuPAN with RViz visualization
Provides an experience closer to the original Gazebo simulation
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
    example_dir = PathJoinSubstitution([neupan_pkg_dir, 'example', 'gazebo_limo'])
    
    # Launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([example_dir, 'config', 'neupan_planner_limo.yaml']),
        description='Path to NeuPAN config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Use real time for this simulation
        description='Use simulation time'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # Enhanced robot simulator with dynamic obstacles and complex environment
    enhanced_robot_simulator = Node(
        package='neupan_ros',
        executable='enhanced_robot_simulator',
        name='enhanced_robot_simulator',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'environment_type': 'complex'},  # Options: 'simple', 'complex', 'dynamic'
            {'num_static_obstacles': 15},
            {'num_dynamic_obstacles': 3},
            {'world_size_x': 8.0},
            {'world_size_y': 8.0},
            {'obstacle_size_min': 0.2},
            {'obstacle_size_max': 0.8},
            {'dynamic_obstacle_speed': 0.3},
            {'laser_range': 5.0},
            {'laser_noise': 0.01}
        ],
        output='screen'
    )
    
    # Robot TF publisher (publishes robot model transforms)
    robot_tf_publisher = Node(
        package='neupan_ros',
        executable='robot_tf_publisher',
        name='robot_tf_publisher', 
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_length': 0.3},
            {'robot_width': 0.2},
            {'robot_height': 0.1}
        ],
        output='screen'
    )
    
    # NeuPAN control node
    neupan_node = Node(
        package='neupan_ros',
        executable='neupan_node',
        name='neupan_control',
        parameters=[
            {'config_file': LaunchConfiguration('config_file')},
            {'map_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'lidar_frame': 'laser_link'},
            {'scan_range': '0.0 5.0'},
            {'scan_angle_range': '-3.14 3.14'},
            {'marker_size': '0.05'},
            {'scan_downsample': '6'},
            {'dune_checkpoint': PathJoinSubstitution([example_dir, 'pretrain_limo', 'model_5000.pth'])},
            {'refresh_initial_path': 'false'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/neupan_cmd_vel', '/cmd_vel'),
            ('/neupan_goal', '/move_base_simple/goal')
        ],
        output='screen'
    )
    
    # RViz2 with enhanced visualization
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([neupan_pkg_dir, 'rviz', 'neupan_enhanced_simulation.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Goal publisher (for testing)
    goal_publisher = Node(
        package='neupan_ros',
        executable='goal_publisher_node',
        name='goal_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'goal_x': 3.5},
            {'goal_y': 3.5},
            {'auto_publish': True},
            {'goal_interval': 30.0}  # Publish new goal every 30 seconds
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        use_sim_time_arg,
        rviz_arg,
        
        # Enhanced simulation components
        enhanced_robot_simulator,
        robot_tf_publisher,
        
        # NeuPAN algorithm (delayed to let simulation start up)
        TimerAction(
            period=3.0,
            actions=[neupan_node]
        ),
        
        # Visualization
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        ),
        
        # Goal publisher (delayed to let everything start up)
        TimerAction(
            period=5.0,
            actions=[goal_publisher]
        ),
    ])