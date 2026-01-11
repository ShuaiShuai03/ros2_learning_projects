#!/usr/bin/env python3
"""
Action client node that calls the Fibonacci action.
This demonstrates the action client pattern in ROS 2.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """An action client that calls the Fibonacci action."""

    def __init__(self):
        """Initialize the FibonacciActionClient."""
        super().__init__('fibonacci_action_client')

        # Create an action client
        # Action name: fibonacci
        # Action type: example_interfaces/action/Fibonacci
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

        self.get_logger().info('Fibonacci action client created')

    def send_goal(self, order):
        """
        Send a goal to compute Fibonacci sequence.

        Args:
            order: The order of the Fibonacci sequence
        """
        # Wait for action server
        self.get_logger().info('等待action服务器...')
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'发送请求，order={order}')

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Register callback for when goal is accepted/rejected
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback when goal is accepted or rejected."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('请求被拒绝')
            return

        self.get_logger().info('请求已接受')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback for receiving feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'收到进度反馈: {feedback.sequence}')

    def get_result_callback(self, future):
        """Callback when final result is received."""
        result = future.result().result
        self.get_logger().info(f'最终结果: {result.sequence}')

        # Shutdown after receiving result
        rclpy.shutdown()


def main(args=None):
    """Main function to run the action client."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = FibonacciActionClient()

    # Send goal
    node.send_goal(5)

    # Keep the node running
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
