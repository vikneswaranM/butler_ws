#!/usr/bin/env python3
"""
Food Delivery Client
Sends delivery orders to the food delivery robot
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from butler_bot.action import FoodDelivery
import sys


class FoodDeliveryClient(Node):
    """Client for sending food delivery orders"""

    def __init__(self):
        super().__init__('food_delivery_client')
        self._action_client = ActionClient(self, FoodDelivery, 'food_delivery')
        self.get_logger().info('Food Delivery Client created')

    def send_order(self, table_numbers, timeout=0.0):
        """Send a delivery order to the robot"""
        self.get_logger().info(f'Sending order for tables: {table_numbers}')
        self.get_logger().info(f'Confirmation timeout: {timeout}s')

        # Wait for server
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Food delivery action server not available')
            return None

        # Create goal
        goal = FoodDelivery.Goal()
        goal.table_numbers = table_numbers
        goal.confirmation_timeout = timeout

        # Send goal
        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Order rejected by server')
            return None

        self.get_logger().info('Order accepted, waiting for completion...')

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        return result

    def feedback_callback(self, feedback_msg):
        """Callback for receiving feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'[Feedback] State: {feedback.current_state} | '
            f'Location: {feedback.current_location} | '
            f'Progress: {feedback.tables_delivered}/{feedback.total_tables}'
        )


def print_usage():
    """Print usage information"""
    print("\n=== Food Delivery Client Usage ===")
    print("Send orders to the butler robot for food delivery\n")
    print("Usage:")
    print("  ros2 run butler_bot food_delivery_client.py <table_numbers> [timeout]\n")
    print("Arguments:")
    print("  table_numbers : Comma-separated list of table numbers (e.g., 1,2,3)")
    print("  timeout       : Optional confirmation timeout in seconds (default: 0 = no confirmation)\n")
    print("Examples:")
    print("  # Scenario 1: Simple delivery to table 1 (no confirmation)")
    print("  ros2 run butler_bot food_delivery_client.py 1")
    print()
    print("  # Scenario 2-4: Single table with 30s confirmation timeout")
    print("  ros2 run butler_bot food_delivery_client.py 1 30")
    print()
    print("  # Scenario 5: Multiple tables (no confirmation)")
    print("  ros2 run butler_bot food_delivery_client.py 1,2,3")
    print()
    print("  # Scenario 6-7: Multiple tables with 30s confirmation timeout")
    print("  ros2 run butler_bot food_delivery_client.py 1,2,3 30")
    print()
    print("During Delivery:")
    print("  # Confirm at kitchen or table")
    print("  ros2 service call /confirm_location butler_bot/srv/Confirmation \"{location: 'kitchen'}\"")
    print("  ros2 service call /confirm_location butler_bot/srv/Confirmation \"{location: 'table_1'}\"")
    print()
    print("  # Cancel entire order (table_number: 0)")
    print("  ros2 service call /cancel_order butler_bot/srv/CancelOrder \"{table_number: 0}\"")
    print()
    print("  # Cancel specific table")
    print("  ros2 service call /cancel_order butler_bot/srv/CancelOrder \"{table_number: 2}\"")
    print()


def main(args=None):
    if len(sys.argv) < 2 or sys.argv[1] in ['-h', '--help', 'help']:
        print_usage()
        return

    rclpy.init(args=args)

    # Parse arguments
    try:
        table_str = sys.argv[1]
        table_numbers = [int(t.strip()) for t in table_str.split(',')]

        timeout = 0.0
        if len(sys.argv) > 2:
            timeout = float(sys.argv[2])

    except ValueError as e:
        print(f"Error parsing arguments: {e}")
        print_usage()
        return

    # Create client and send order
    client = FoodDeliveryClient()

    try:
        result = client.send_order(table_numbers, timeout)

        if result:
            print("\n=== Delivery Result ===")
            print(f"Success: {result.success}")
            print(f"Message: {result.message}")
            print(f"Delivered tables: {result.delivered_tables}")
            print(f"Skipped tables: {result.skipped_tables}")
        else:
            print("Failed to get result from server")

    except KeyboardInterrupt:
        client.get_logger().info('Interrupted by user')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
