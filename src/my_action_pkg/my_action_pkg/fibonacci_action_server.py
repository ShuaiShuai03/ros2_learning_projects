#!/usr/bin/env python3
"""
Action server node that provides a Fibonacci action.
This demonstrates the action server pattern in ROS 2.
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """An action server that computes Fibonacci sequences."""

    def __init__(self):
        """Initialize the FibonacciActionServer."""
        super().__init__('fibonacci_action_server')

        # Create an action server
        # Action name: fibonacci
        # Action type: example_interfaces/action/Fibonacci
        # Callback: execute_callback
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci action server is ready!')

    def execute_callback(self, goal_handle):
        """
        Execute the Fibonacci action.

        Args:
            goal_handle: The goal handle for this action

        Returns:
            result: The final Fibonacci sequence
        """
        self.get_logger().info(f'执行目标，order={goal_handle.request.order}')

        # Initialize feedback message
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('目标被取消')
                return Fibonacci.Result()

            # Compute next Fibonacci number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            self.get_logger().info(f'发布反馈: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

            # Simulate computation time
            time.sleep(1)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Create result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'返回结果: {result.sequence}')

        return result


def main(args=None):
    """Main function to run the action server."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = FibonacciActionServer()

    # Keep the node running
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
