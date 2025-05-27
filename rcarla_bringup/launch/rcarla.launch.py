from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='traffic_interface',
            executable='traffic_interface',
            name='traffic_interface',
            output='screen',
            parameters=[{'num_npc': 0}],
        ),
        Node(
            package='drive_interface',
            executable='drive_interface',
            name='drive_interface',
            output='screen',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
    ])
