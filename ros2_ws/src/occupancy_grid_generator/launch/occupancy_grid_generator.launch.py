#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('occupancy_grid_generator'),
            'config',
            'occupancy_grid_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Occupancy grid generator node
    occupancy_grid_node = Node(
        package='occupancy_grid_generator',
        executable='occupancy_grid_generator_node',
        name='occupancy_grid_generator',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        occupancy_grid_node
    ])
