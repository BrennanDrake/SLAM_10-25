#!/usr/bin/env python3
"""
RTAB-Map Launch File for Intel RealSense D455
Phase 4: Visual SLAM Session

This launch file configures RTAB-Map for RGB-D SLAM with the D455 camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Environment setup
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Arguments
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='false',
            description='Delete existing database on start'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Set to true for localization mode (don\'t update map)'
        ),
        
        # RTAB-Map Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # Core settings
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'approx_sync': True,
                
                # Database
                'database_path': '~/.ros/rtabmap_d455.db',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # SLAM parameters
                'RGBD/ProximityBySpace': 'true',
                'RGBD/ProximityPathMaxNeighbors': '10',
                'RGBD/AngularUpdate': '0.01',  # Update every ~0.6 degrees
                'RGBD/LinearUpdate': '0.01',   # Update every 1 cm
                'Reg/Strategy': '1',           # Visual registration
                'Reg/Force3DoF': 'false',      # Allow full 6DOF
                'Vis/MinInliers': '15',        # Minimum feature matches
                'Vis/InlierDistance': '0.1',   # RANSAC threshold
                
                # Loop closure
                'Mem/STMSize': '30',           # Short-term memory
                'Kp/MaxFeatures': '400',       # Features per image
                'Kp/DetectorStrategy': '6',    # 6=ORB (fast)
                'GFTT/MinDistance': '7',       # Minimum distance between features
                
                # Optimization
                'Optimizer/Strategy': '1',     # g2o
                'Optimizer/Iterations': '20',
                'RGBD/OptimizeFromGraphEnd': 'false',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            arguments=[
                '-d' if LaunchConfiguration('delete_db_on_start') == 'true' else '',
                '--Mem/IncrementalMemory', 'true' if LaunchConfiguration('localization') == 'false' else 'false',
            ]
        ),
        
        # Visualization Node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_odom_info': True,
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ]
        ),
    ])
