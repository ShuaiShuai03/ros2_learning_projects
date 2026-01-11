#!/usr/bin/env python3
"""
Hello World ROS 2 node using OOP (Object-Oriented Programming) style
This demonstrates the class-based approach to creating ROS 2 nodes.
"""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    """A simple Hello World node that demonstrates ROS 2 node creation."""

    def __init__(self):
        """Initialize the HelloNode."""
        super().__init__('hello_node_class')

        # Log initial messages
        self.get_logger().info('Hello from HelloNode class!')
        self.get_logger().info('Node initialized: %s' % self.get_name())

        # Create a timer that calls our callback every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Timer callback function that runs periodically."""
        self.counter += 1
        self.get_logger().info(f'Hello ROS 2! Count: {self.counter}')


def main(args=None):
    """Main function to run the hello node."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = HelloNode()

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
