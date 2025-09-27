from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    node_name_prefix_arg = DeclareLaunchArgument(
        'name_prefix', default_value='template_pkg',
        description='Prefix for node names'
    )
    topic_arg = DeclareLaunchArgument(
        'topic', default_value='chatter',
        description='Topic to publish/subscribe String messages'
    )

    prefix = LaunchConfiguration('name_prefix')
    topic = LaunchConfiguration('topic')

    talker = Node(
        package='template_pkg',
        executable='talker',
        name=[prefix, '_talker'],
        remappings=[('chatter', topic)],
        output='screen'
    )

    listener = Node(
        package='template_pkg',
        executable='listener',
        name=[prefix, '_listener'],
        remappings=[('chatter', topic)],
        output='screen'
    )

    return LaunchDescription([
        node_name_prefix_arg,
        topic_arg,
        talker,
        listener,
    ])
