#!/usr/bin/env python3
"""
Launch file that starts multiple nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with multiple nodes."""
    return LaunchDescription([
        # Launch ticker node
        Node(
            package='my_second_pkg',
            executable='ticker',
            name='ticker_node',
            output='screen',
        ),

        # Launch listener node
        Node(
            package='my_second_pkg',
            executable='listener',
            name='listener_node',
            output='screen',
        ),
    ])
