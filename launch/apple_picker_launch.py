from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Control node - supervisor subscriber
        Node(
            package='control_node',
            executable='supervisor_sub',
            name='supervisor_sub',
            output='screen'
        ),

        # User interface - supervisor publisher
        Node(
            package='user_interface',
            executable='supervisor_pub',
            name='supervisor_pub',
            output='screen'
        ),

        # Perception server
        Node(
            package='perception_pkg',
            executable='perception_server',
            name='perception_server',
            output='screen'
        )
    ])