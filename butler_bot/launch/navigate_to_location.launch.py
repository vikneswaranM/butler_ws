from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the navigate to location action server."""
    
    navigate_to_location_server = Node(
        package='butler_bot',
        executable='navigate_to_location_server.py',
        name='navigate_to_location_action_server',
        output='screen'
    )
    
    return LaunchDescription([
        navigate_to_location_server
    ])

