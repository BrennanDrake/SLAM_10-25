#!/usr/bin/env python3
"""
Test launch file for Hector SLAM with simulated laser data.

This launch file starts:
1. Fake scan publisher (from Phase 1) to simulate laser data
2. Hector SLAM processor node
3. Static transform publisher for base_link -> laser
4. RViz for visualization (optional)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Get package directories
    hector_slam_dir = get_package_share_directory('hector_slam_custom')
    occupancy_grid_dir = get_package_share_directory('occupancy_grid_generator')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        )
    )
    
    # 1. Fake scan publisher (simulates laser data)
    # This uses the fake scan publisher from Phase 1
    fake_scan_node = Node(
        package='occupancy_grid_generator',
        executable='fake_scan_publisher',
        name='fake_scan_publisher',
        output='screen',
        parameters=[{
            'scan_topic': 'scan',
            'frame_id': 'laser',
            'range_min': 0.1,
            'range_max': 10.0,
            'angle_min': -3.14159,  # -pi
            'angle_max': 3.14159,   # pi
            'angle_increment': 0.0174533,  # ~1 degree
            'scan_time': 0.1,
            'range_noise': 0.01,
            'publish_rate': 10.0
        }]
    )
    
    # 2. Hector SLAM processor node
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
            'map_size': 100.0,       # 100m x 100m
            'map_update_distance_threshold': 0.4,  # meters
            'map_update_angle_threshold': 0.2,     # radians
        }],
        remappings=[
            ('scan', 'scan'),
            ('map', 'map'),
            ('pose', 'slam_pose')
        ]
    )
    
    # 3. Static transform publisher for base_link -> laser
    # This defines where the laser is mounted on the robot
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    
    # 4. RViz for visualization (optional)
    rviz_config_file = os.path.join(hector_slam_dir, 'config', 'slam_visualization.rviz')
    
    # Check if config file exists, if not use default
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        condition=IfCondition(use_rviz)
    )
    
    # Add all nodes to launch description
    ld.add_action(fake_scan_node)
    ld.add_action(hector_slam_node)
    ld.add_action(static_tf_node)
    # ld.add_action(rviz_node)  # Uncomment when RViz config is ready
    
    # Log startup message
    ld.add_action(
        ExecuteProcess(
            cmd=['echo', '========================================'],
            output='screen'
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=['echo', 'Starting Hector SLAM test with simulated data'],
            output='screen'
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=['echo', 'Topics: /scan (input), /map (output), /slam_pose (output)'],
            output='screen'
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=['echo', '========================================'],
            output='screen'
        )
    )
    
    return ld
