#!/usr/bin/env python3
"""
Launch file for testing Hector SLAM with static environment.

This simulates a robot in a rectangular room with consistent walls,
providing repeatable test conditions for verifying scan matching.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    room_width = LaunchConfiguration('room_width', default='10.0')
    room_height = LaunchConfiguration('room_height', default='8.0')
    robot_x = LaunchConfiguration('robot_x', default='0.0')
    robot_y = LaunchConfiguration('robot_y', default='0.0')
    add_noise = LaunchConfiguration('add_noise', default='true')
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'room_width', default_value='10.0',
        description='Width of simulated room in meters'))
    
    ld.add_action(DeclareLaunchArgument(
        'room_height', default_value='8.0',
        description='Height of simulated room in meters'))
    
    ld.add_action(DeclareLaunchArgument(
        'robot_x', default_value='0.0',
        description='Initial robot X position'))
    
    ld.add_action(DeclareLaunchArgument(
        'robot_y', default_value='0.0',
        description='Initial robot Y position'))
    
    ld.add_action(DeclareLaunchArgument(
        'add_noise', default_value='true',
        description='Add noise to scan data'))
    
    # Log configuration
    ld.add_action(LogInfo(msg='==========================================='))
    ld.add_action(LogInfo(msg='Starting Hector SLAM with static environment'))
    ld.add_action(LogInfo(msg='Room: 10x8m, Robot at origin'))
    ld.add_action(LogInfo(msg='==========================================='))
    
    # 1. Static scan publisher (simulates rectangular room)
    static_scan_node = Node(
        package='hector_slam_custom',
        executable='static_scan_publisher',
        name='static_scan_publisher',
        output='screen',
        parameters=[{
            'scan_topic': 'scan',
            'publish_rate': 10.0,
            'room_width': room_width,
            'room_height': room_height,
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_theta': 0.0,
            'add_noise': add_noise,
            'noise_stddev': 0.01
        }]
    )
    
    # 2. Hector SLAM processor
    hector_slam_node = Node(
        package='hector_slam_custom',
        executable='hector_slam_custom_node',
        name='hector_slam_processor',
        output='screen',
        parameters=[{
            # Frame IDs
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'laser_frame': 'laser',
            
            # Publishing rates
            'map_publish_rate': 2.0,
            'pose_publish_rate': 20.0,
            
            # Scan matching parameters
            'scan_match_threshold': 0.01,
            'max_scan_match_iterations': 20,
            
            # Map parameters
            'map_resolution': 0.05,  # 5cm per cell
            'map_size': 20.0,        # 20m x 20m (smaller for testing)
            'map_update_distance_threshold': 0.1,  # Update more frequently
            'map_update_angle_threshold': 0.1,     # radians
        }]
    )
    
    # 3. Static transform: base_link -> laser
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )
    
    # 4. Monitor node (our custom monitor)
    monitor_node = Node(
        package='hector_slam_custom',
        executable='monitor_slam.py',
        name='slam_monitor',
        output='screen'
    )
    
    # Add all nodes
    ld.add_action(static_scan_node)
    ld.add_action(hector_slam_node)
    ld.add_action(static_tf_node)
    ld.add_action(monitor_node)
    
    return ld
