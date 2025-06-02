from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():

    use_joystick = os.environ.get("USE_JOYSTICK", "false").lower() == "true"

    actions = [
        Node(
            package='traffic_interface',
            executable='traffic_interface',
            name='traffic_interface',
            output='screen',
            parameters=["/opt/ws/src/cfg/ros2_config.yaml"],
        ),
        Node(
            package='drive_interface',
            executable='drive_interface',
            name='drive_interface',
            output='screen',
            parameters=["/opt/ws/src/cfg/ros2_config.yaml"],
        ),
        Node(
            package='drive_interface',
            executable='physics_interface',
            name='physics_interface',
            output='screen',
        )
        ]

    if use_joystick:
        actions.append(
            Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        )
        )

    return LaunchDescription(actions)
