#!/usr/bin/env python3
"""
Food Delivery Robot Node
Handles complex food delivery scenarios with state machine logic
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from butler_bot.action import FoodDelivery, NavigateToLocation
from butler_bot.srv import Confirmation, CancelOrder
from enum import Enum
import threading
import time


class RobotState(Enum):
    """Robot states for the delivery state machine"""
    IDLE = "idle"
    GOING_TO_KITCHEN = "going_to_kitchen"
    AT_KITCHEN = "at_kitchen"
    GOING_TO_TABLE = "going_to_table"
    AT_TABLE = "at_table"
    RETURNING_TO_KITCHEN = "returning_to_kitchen"
    GOING_HOME = "going_home"
    AT_HOME = "at_home"


class FoodDeliveryRobot(Node):
    """Main food delivery robot node with state machine"""

    def __init__(self):
        super().__init__('food_delivery_robot')

        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()

        # State variables
        self.current_state = RobotState.IDLE
        self.current_order = None
        self.delivered_tables = []
        self.skipped_tables = []
        self.pending_tables = []
        self.cancelled_tables = []
        self.confirmation_received = False
        self.navigation_cancelled = False
        self.order_goal_handle = None

        # Lock for thread-safe state access
        self.state_lock = threading.Lock()

        # Create action server for food delivery
        self._action_server = ActionServer(
            self,
            FoodDelivery,
            'food_delivery',
            self.execute_delivery_callback,
            callback_group=self.callback_group
        )

        # Create action client for navigation
        self._nav_client = ActionClient(
            self,
            NavigateToLocation,
            'navigate_to_location',
            callback_group=self.callback_group
        )

        # Create service for confirmation
        self._confirmation_service = self.create_service(
            Confirmation,
            'confirm_location',
            self.confirmation_callback,
            callback_group=self.callback_group
        )

        # Create service for order cancellation
        self._cancel_service = self.create_service(
            CancelOrder,
            'cancel_order',
            self.cancel_order_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Food Delivery Robot started')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  - Send delivery order via action: /food_delivery')
        self.get_logger().info('  - Confirm location: ros2 service call /confirm_location')
        self.get_logger().info('  - Cancel table order: ros2 service call /cancel_order')

    def execute_delivery_callback(self, goal_handle):
        """Main execution callback for food delivery action"""
        self.get_logger().info('=== New Food Delivery Order Received ===')

        with self.state_lock:
            self.order_goal_handle = goal_handle
            self.current_order = goal_handle.request
            self.pending_tables = list(self.current_order.table_numbers)
            self.delivered_tables = []
            self.skipped_tables = []
            self.cancelled_tables = []
            self.confirmation_received = False
            self.navigation_cancelled = False

        timeout = self.current_order.confirmation_timeout
        table_numbers = self.current_order.table_numbers

        self.get_logger().info(f'Tables to deliver: {table_numbers}')
        self.get_logger().info(f'Confirmation timeout: {timeout}s (0 = no confirmation)')

        # Start delivery workflow
        result = FoodDelivery.Result()

        try:
            # Step 1: Go to kitchen
            if not self._navigate_to_location('kitchen', 'GOING_TO_KITCHEN'):
                result.success = False
                result.message = "Failed to reach kitchen"
                result.delivered_tables = self.delivered_tables
                result.skipped_tables = self.skipped_tables
                self._return_home()
                return result

            # Check if cancelled while going to kitchen (Scenario 4)
            if self.navigation_cancelled:
                self.get_logger().warn('Order cancelled while going to kitchen, returning home')
                self._return_home()
                result.success = False
                result.message = "Order cancelled while going to kitchen"
                result.delivered_tables = self.delivered_tables
                result.skipped_tables = self.skipped_tables
                return result

            # Step 2: Wait for confirmation at kitchen
            self._update_state(RobotState.AT_KITCHEN, 'kitchen')

            if timeout > 0:
                confirmed = self._wait_for_confirmation('kitchen', timeout)

                if not confirmed:
                    # Scenario 2 & 3a: No confirmation at kitchen, return home
                    self.get_logger().warn('No confirmation at kitchen, timeout. Returning home')
                    self._return_home()
                    result.success = False
                    result.message = "Timeout at kitchen, no confirmation received"
                    result.delivered_tables = self.delivered_tables
                    result.skipped_tables = self.pending_tables
                    return result

                self.get_logger().info('Kitchen confirmation received, proceeding with deliveries')
            else:
                self.get_logger().info('No confirmation required, proceeding with deliveries')

            # Step 3: Deliver to each table
            kitchen_confirmation_received = (timeout == 0 or self.confirmation_received)

            for table_num in table_numbers:
                # Check if this table was cancelled
                if table_num in self.cancelled_tables:
                    self.get_logger().info(f'Table {table_num} was cancelled, skipping')
                    self.skipped_tables.append(table_num)
                    if table_num in self.pending_tables:
                        self.pending_tables.remove(table_num)
                    continue

                # Navigate to table
                table_location = f'table_{table_num}'
                self.get_logger().info(f'Delivering to {table_location}')

                if not self._navigate_to_location(table_location, 'GOING_TO_TABLE', table_num):
                    self.get_logger().warn(f'Failed to reach {table_location}')
                    self.skipped_tables.append(table_num)
                    if table_num in self.pending_tables:
                        self.pending_tables.remove(table_num)
                    continue

                # Check if cancelled while going to table (Scenario 4)
                if self.navigation_cancelled or table_num in self.cancelled_tables:
                    self.get_logger().warn(f'Order cancelled while going to table {table_num}')
                    self.skipped_tables.append(table_num)
                    if table_num in self.pending_tables:
                        self.pending_tables.remove(table_num)
                    # Return to kitchen then home (Scenario 4)
                    self._navigate_to_location('kitchen', 'RETURNING_TO_KITCHEN')
                    self._return_home()
                    result.success = False
                    result.message = f"Order cancelled while going to table {table_num}"
                    result.delivered_tables = self.delivered_tables
                    result.skipped_tables = self.skipped_tables
                    return result

                # At table, wait for confirmation
                self._update_state(RobotState.AT_TABLE, table_location, table_num)

                if timeout > 0:
                    self.confirmation_received = False
                    confirmed = self._wait_for_confirmation(table_location, timeout)

                    if not confirmed:
                        # Scenario 3b & 6: No confirmation at table
                        self.get_logger().warn(f'No confirmation at {table_location}, timeout')
                        self.skipped_tables.append(table_num)
                        if table_num in self.pending_tables:
                            self.pending_tables.remove(table_num)

                        # Check if this is the last table
                        remaining = [t for t in table_numbers if t not in self.delivered_tables and t not in self.skipped_tables]
                        if not remaining:
                            # Last table, return to kitchen if food was received (Scenario 3b, 6)
                            if kitchen_confirmation_received:
                                self.get_logger().info('Last table with timeout, returning to kitchen first')
                                self._navigate_to_location('kitchen', 'RETURNING_TO_KITCHEN')
                        continue

                    self.get_logger().info(f'Table {table_num} confirmation received')
                else:
                    self.get_logger().info(f'No confirmation required at table {table_num}')

                # Mark as delivered
                self.delivered_tables.append(table_num)
                if table_num in self.pending_tables:
                    self.pending_tables.remove(table_num)

                self._publish_feedback(f'Delivered to table {table_num}', table_location, table_num)

            # Step 4: Check if we need to return to kitchen
            # Scenario 6: If any table had timeout, return to kitchen first
            if self.skipped_tables and kitchen_confirmation_received:
                self.get_logger().info('Some deliveries incomplete, returning to kitchen first')
                self._navigate_to_location('kitchen', 'RETURNING_TO_KITCHEN')

            # Step 5: Return home
            self._return_home()

            # Prepare result
            if self.delivered_tables:
                result.success = True
                result.message = f"Delivery completed. Delivered: {len(self.delivered_tables)}, Skipped: {len(self.skipped_tables)}"
            else:
                result.success = False
                result.message = "No deliveries completed"

            result.delivered_tables = self.delivered_tables
            result.skipped_tables = self.skipped_tables

            self.get_logger().info('=== Delivery Order Complete ===')
            self.get_logger().info(f'Delivered: {self.delivered_tables}')
            self.get_logger().info(f'Skipped: {self.skipped_tables}')

            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Error during delivery: {e}')
            self._return_home()
            result.success = False
            result.message = f"Error: {str(e)}"
            result.delivered_tables = self.delivered_tables
            result.skipped_tables = self.skipped_tables
            goal_handle.abort()
            return result

    def _navigate_to_location(self, location, state_name, table_num=0):
        """Navigate to a location and return success status"""
        self.get_logger().info(f'Navigating to {location}...')
        self._update_state(RobotState[state_name], location, table_num)

        # Wait for navigation server
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return False

        # Create and send goal
        nav_goal = NavigateToLocation.Goal()
        nav_goal.location_name = location

        send_goal_future = self._nav_client.send_goal_async(nav_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        if not send_goal_future.done():
            self.get_logger().error(f'Failed to send navigation goal to {location}')
            return False

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation goal to {location} rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()

        # Poll for result while checking for cancellation
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if result_future.done():
                result = result_future.result().result
                if result.success:
                    self.get_logger().info(f'Successfully reached {location}')
                    return True
                else:
                    self.get_logger().error(f'Failed to reach {location}: {result.message}')
                    return False

            # Check for cancellation
            if self.navigation_cancelled:
                self.get_logger().warn('Navigation cancelled by user')
                goal_handle.cancel_goal_async()
                return False

            # Check if specific table was cancelled
            if table_num > 0 and table_num in self.cancelled_tables:
                self.get_logger().warn(f'Table {table_num} cancelled during navigation')
                goal_handle.cancel_goal_async()
                return False

        return False

    def _return_home(self):
        """Return robot to home position"""
        self.get_logger().info('Returning to home position...')
        self._navigate_to_location('home_pose', 'GOING_HOME')
        self._update_state(RobotState.AT_HOME, 'home_pose')
        self.get_logger().info('Robot is back at home')

    def _wait_for_confirmation(self, location, timeout):
        """Wait for confirmation with timeout"""
        self.get_logger().info(f'Waiting for confirmation at {location} (timeout: {timeout}s)...')
        self.confirmation_received = False

        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.confirmation_received:
                return True

            # Check for cancellation
            if self.navigation_cancelled:
                return False

        self.get_logger().warn(f'Confirmation timeout at {location}')
        return False

    def _update_state(self, state, location='', table_num=0):
        """Update robot state and publish feedback"""
        with self.state_lock:
            self.current_state = state
        self._publish_feedback(state.value, location, table_num)

    def _publish_feedback(self, state_msg, location='', table_num=0):
        """Publish feedback to action client"""
        if self.order_goal_handle is None:
            return

        feedback = FoodDelivery.Feedback()
        feedback.current_state = state_msg
        feedback.current_location = location
        feedback.current_table = table_num
        feedback.tables_delivered = len(self.delivered_tables)
        feedback.total_tables = len(self.current_order.table_numbers) if self.current_order else 0

        self.order_goal_handle.publish_feedback(feedback)

    def confirmation_callback(self, request, response):
        """Service callback for location confirmation"""
        location = request.location
        self.get_logger().info(f'Confirmation received for location: {location}')

        with self.state_lock:
            self.confirmation_received = True

        response.confirmed = True
        response.message = f'Confirmation received for {location}'
        return response

    def cancel_order_callback(self, request, response):
        """Service callback for cancelling specific table order"""
        table_num = request.table_number
        self.get_logger().warn(f'Cancellation request for table {table_num}')

        with self.state_lock:
            if table_num in self.pending_tables or table_num == 0:
                if table_num == 0:
                    # Cancel entire order
                    self.navigation_cancelled = True
                    response.message = 'Entire order cancelled'
                else:
                    self.cancelled_tables.append(table_num)
                    response.message = f'Table {table_num} cancelled'
                response.success = True
            else:
                response.success = False
                response.message = f'Table {table_num} not in current order or already processed'

        return response


def main(args=None):
    rclpy.init(args=args)

    robot = FoodDeliveryRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
