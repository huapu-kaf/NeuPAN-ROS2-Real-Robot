#!/usr/bin/env python3
"""
ROS2 launch file for NeuPAN Gazebo simulation with LIMO robot
Converted from ROS1 launch files in example/gazebo_limo/launch/
"""

import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    neupan_pkg_dir = FindPackageShare('neupan_ros')
    example_dir = PathJoinSubstitution([neupan_pkg_dir, 'example', 'gazebo_limo'])
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([example_dir, 'world', 'gazebo_world_complex_10.world']),
        description='Path to Gazebo world file'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([example_dir, 'config', 'neupan_planner_limo.yaml']),
        description='Path to NeuPAN config file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # Robot spawn arguments
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0') 
    z_arg = DeclareLaunchArgument('z', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    
    # Gazebo server (gzserver)
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             LaunchConfiguration('world_file')],
        output='screen'
    )
    
    # Gazebo client (gzclient)
    gzclient = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('gui')),
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot description - LIMO URDF
    robot_description_content = Command([
        'xacro ', PathJoinSubstitution([
            example_dir, 'limo_ros', 'limo_description', 'urdf', 'limo_four_diff.xacro'
        ])
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': robot_description_content}
        ],
        output='screen'
    )
    
    # Joint state publisher  
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Spawn LIMO robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_limo_model',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'limo',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'), 
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
            ('/scan', '/limo/scan'),
            ('/neupan_cmd_vel', '/cmd_vel'),
            ('/neupan_goal', '/move_base_simple/goal')
        ],
        output='screen'
    )
    
    # RViz2
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([example_dir, 'rviz', 'limo_gazebo.rviz'])],
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
            {'goal_x': 3.0},
            {'goal_y': 3.0}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        world_arg,
        config_file_arg,
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        
        # Gazebo
        gzserver,
        TimerAction(
            period=2.0,
            actions=[gzclient]
        ),
        
        # Robot
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(
            period=3.0,
            actions=[spawn_robot]
        ),
        
        # NeuPAN algorithm
        TimerAction(
            period=5.0,
            actions=[neupan_node]
        ),
        
        # Visualization
        TimerAction(
            period=4.0,
            actions=[rviz_node]
        ),
        
        # Goal publisher (delayed to let everything start up)
        TimerAction(
            period=8.0,
            actions=[goal_publisher]
        ),
    ])