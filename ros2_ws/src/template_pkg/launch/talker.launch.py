from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='template_pkg_talker',
        description='Name of the talker node'
    )
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='chatter',
        description='Topic to publish String messages on'
    )

    node_name = LaunchConfiguration('node_name')
    topic = LaunchConfiguration('topic')

    talker = Node(
        package='template_pkg',
        executable='talker',
        name=node_name,
        remappings=[('chatter', topic)],
        output='screen'
    )

    return LaunchDescription([
        node_name_arg,
        topic_arg,
        talker,
    ])
