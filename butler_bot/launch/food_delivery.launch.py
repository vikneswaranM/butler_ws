#!/usr/bin/env python3
"""
Launch file for the complete food delivery system
Starts navigation action server and food delivery robot
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch navigation action server
        Node(
            package='butler_bot',
            executable='navigate_to_location_server.py',
            name='navigate_to_location_server',
            output='screen',
            emulate_tty=True,
        ),

        # Launch food delivery robot
        Node(
            package='butler_bot',
            executable='food_delivery_robot.py',
            name='food_delivery_robot',
            output='screen',
            emulate_tty=True,
        ),
    ])
