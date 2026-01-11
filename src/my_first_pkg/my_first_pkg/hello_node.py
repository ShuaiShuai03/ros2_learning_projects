#!/usr/bin/env python3
"""
Simple Hello World ROS 2 node (functional style)
This is the simplest way to create a ROS 2 node in Python.
"""

import rclpy


def main(args=None):
    """Main function to run the hello node."""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create a node
    node = rclpy.create_node('hello_node')

    # Log a message
    node.get_logger().info('Hello, ROS 2! 我是你的第一个节点！')
    node.get_logger().info('This is my first ROS 2 node!')
    node.get_logger().info('Node name: %s' % node.get_name())

    # Keep the node running for a bit
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
