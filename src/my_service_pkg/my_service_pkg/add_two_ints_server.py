#!/usr/bin/env python3
"""
Service server node that provides an AddTwoInts service.
This demonstrates the service server pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """A service server that adds two integers."""

    def __init__(self):
        """Initialize the AddTwoIntsServer."""
        super().__init__('add_two_ints_server')

        # Create a service
        # Service name: add_two_ints
        # Service type: example_interfaces/srv/AddTwoInts
        # Callback: add_two_ints_callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('AddTwoInts service server is ready!')

    def add_two_ints_callback(self, request, response):
        """
        Service callback that adds two integers.

        Args:
            request: AddTwoInts.Request with fields 'a' and 'b'
            response: AddTwoInts.Response with field 'sum'

        Returns:
            response: The response with the sum
        """
        # Perform the addition
        response.sum = request.a + request.b

        # Log the operation
        self.get_logger().info(
            f'收到请求: {request.a} + {request.b} = {response.sum}'
        )

        return response


def main(args=None):
    """Main function to run the service server."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = AddTwoIntsServer()

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
