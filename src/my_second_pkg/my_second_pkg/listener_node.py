#!/usr/bin/env python3
"""
Subscriber node that listens to messages on a topic.
This demonstrates the basic subscriber pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """A node that subscribes to messages from a topic."""

    def __init__(self):
        """Initialize the ListenerNode."""
        super().__init__('listener_node')

        # Create a subscriber
        # Topic: /chatter
        # Message type: std_msgs/msg/String
        # Callback: listener_callback
        # Queue size: 10
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

        # Prevent unused variable warning
        self.subscription

        self.get_logger().info('Listener node started! Subscribed to /chatter')

    def listener_callback(self, msg):
        """Callback function that processes received messages."""
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """Main function to run the listener node."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = ListenerNode()

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
