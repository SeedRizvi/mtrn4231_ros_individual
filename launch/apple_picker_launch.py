from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('visualisation_pkg')
    rviz_config_path = os.path.join(pkg_share, 'apple_detection_config.rviz')
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
        ),

        # Apple data transfer node
        Node(
            package='perception_pkg',
            executable='apple_data_transfer',
            name='apple_data_transfer',
            output='screen'
        ),

        # RViz wrapper
        Node(
            package='visualisation_pkg',
            executable='rviz_wrapper',
            name='rviz_wrapper',
            output='screen'
        ),

        # RViz with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])