#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='Gazebo world to load'
    )

    # Static transform from map to odom (for simulation without SLAM)
    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # TurtleBot3 simulation (if available)
    try:
        turtlebot3_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    except:
        # Fallback: create a simple laser scan simulator
        turtlebot3_gazebo = Node(
            package='occupancy_grid_generator',
            executable='fake_scan_publisher',
            name='fake_scan_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        )

    # Occupancy grid generator
    occupancy_grid_node = Node(
        package='occupancy_grid_generator',
        executable='occupancy_grid_generator_node',
        name='occupancy_grid_generator',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('occupancy_grid_generator'),
                'config',
                'occupancy_grid_params.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # RViz2 for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('occupancy_grid_generator'),
        'config',
        'occupancy_grid_rviz.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        static_transform_map_odom,
        turtlebot3_gazebo,
        occupancy_grid_node,
        rviz_node
    ])
