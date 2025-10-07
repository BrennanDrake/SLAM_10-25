#!/usr/bin/env python3
"""
RTAB-Map Launch File for Intel RealSense D455
Phase 4: Visual SLAM Session

This launch file configures RTAB-Map for RGB-D SLAM with the D455 camera.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='false',
            description='Delete existing database on start'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Set to true for localization mode'
        ),
        
        # RTAB-Map Visual Odometry Node (computes odometry from camera)
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,
                'subscribe_rgbd': False,
                
                # Visual odometry parameters
                'Odom/Strategy': '0',          # 0=Frame-to-Map, 1=Frame-to-Frame
                'Vis/MinInliers': '12',
                'Vis/InlierDistance': '0.1',
                'Vis/MaxDepth': '4.0',
                'OdomF2M/MaxSize': '1000',
                'Odom/FillInfoData': 'true',
                'Odom/ResetCountdown': '1',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ]
        ),
        
        # RTAB-Map SLAM Node (uses odometry from above)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                # Frame IDs
                'frame_id': 'camera_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                
                # Subscribe topics
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': False,
                'subscribe_odom_info': True,
                'approx_sync': True,
                
                # Database
                'database_path': '~/.ros/rtabmap_d455.db',
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                
                # Registration (frame-to-frame matching)
                'Reg/Strategy': '1',           # 1=Visual, 0=ICP
                'Reg/Force3DoF': 'false',      # Allow full 6DOF
                'Vis/MinInliers': '15',        # Min features for valid match
                'Vis/InlierDistance': '0.1',   # RANSAC threshold (meters)
                'Vis/MaxDepth': '4.0',         # Max depth for features (meters)
                
                # Feature detection (ORB)
                'Kp/MaxFeatures': '400',       # Features per keyframe
                'Kp/DetectorStrategy': '6',    # 6=ORB, 0=SURF, 1=SIFT
                'GFTT/MinDistance': '7',       # Pixel spacing between features
                
                # Memory management
                'Mem/STMSize': '30',           # Short-term memory size
                'Mem/ReduceGraph': 'false',    # Keep full graph initially
                
                # Loop closure detection
                'Rtabmap/DetectionRate': '1',  # Check for loops every frame
                'RGBD/LoopClosureReextractFeatures': 'true',
                
                # Graph optimization
                'Optimizer/Strategy': '1',     # 1=g2o, 2=GTSAM
                'Optimizer/Iterations': '20',
                'RGBD/OptimizeFromGraphEnd': 'false',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ],
            arguments=[
                '--delete_db_on_start' if LaunchConfiguration('delete_db_on_start') == 'true' else '',
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
