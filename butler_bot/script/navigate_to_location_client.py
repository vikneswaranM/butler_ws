#!/usr/bin/env python3
"""
Example action client for navigating to predefined locations.
This script demonstrates how to use the NavigateToLocation action server.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from butler_bot.action import NavigateToLocation  # type: ignore
import sys


class NavigateToLocationClient(Node):
    """Action client for navigating to predefined locations."""
    
    def __init__(self):
        super().__init__('navigate_to_location_client')
        self._action_client = ActionClient(self, NavigateToLocation, 'navigate_to_location')
        self.get_logger().info('NavigateToLocation action client created')
    
    def wait_for_server(self, timeout_sec=10.0):
        """Wait for the action server to be available."""
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('Action server not available')
            return False
        self.get_logger().info('Action server is available')
        return True
    
    def send_goal(self, location_name):
        """Send a goal to navigate to a location."""
        goal_msg = NavigateToLocation.Goal()
        goal_msg.location_name = location_name
        
        self.get_logger().info(f'Sending goal to navigate to: {location_name}')
        
        # Send goal and return future
        return self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
    
    def feedback_callback(self, feedback_msg):
        """Callback for receiving feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: {feedback.status}, '
            f'Distance remaining: {feedback.distance_remaining:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: navigate_to_location_client.py <location_name>")
        print("Available locations: home_pose, kitchen, table_1, table_2, table_3")
        return
    
    location_name = sys.argv[1]
    
    client = NavigateToLocationClient()
    
    # Wait for server
    if not client.wait_for_server():
        client.destroy_node()
        rclpy.shutdown()
        return
    
    # Send goal
    future = client.send_goal(location_name)
    
    try:
        rclpy.spin_until_future_complete(client, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            client.get_logger().error('Goal rejected')
            client.destroy_node()
            rclpy.shutdown()
            return
        
        client.get_logger().info('Goal accepted, waiting for result...')
        
        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(client, result_future)
        
        result = result_future.result().result
        
        if result.success:
            client.get_logger().info(f'Success: {result.message}')
        else:
            client.get_logger().error(f'Failed: {result.message}')
            
    except KeyboardInterrupt:
        client.get_logger().info('Interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

