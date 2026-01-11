#!/usr/bin/env python3
"""
Publisher node that publishes a counter to a topic.
This demonstrates the basic publisher pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TickerNode(Node):
    """A node that publishes a counter message periodically."""

    def __init__(self):
        """Initialize the TickerNode."""
        super().__init__('ticker_node')

        # Create a publisher
        # Topic: /chatter
        # Message type: std_msgs/msg/String
        # Queue size: 10
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer that calls our callback every 1 second
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize counter
        self.counter = 0

        self.get_logger().info('Ticker node started! Publishing to /chatter')

    def timer_callback(self):
        """Timer callback that publishes a message."""
        # Create a message
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter
        self.counter += 1


def main(args=None):
    """Main function to run the ticker node."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = TickerNode()

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
