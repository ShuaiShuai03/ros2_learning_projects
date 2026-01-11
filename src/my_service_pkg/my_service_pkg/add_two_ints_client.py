#!/usr/bin/env python3
"""
Service client node that calls the AddTwoInts service.
This demonstrates the service client pattern in ROS 2.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """A service client that calls the AddTwoInts service."""

    def __init__(self):
        """Initialize the AddTwoIntsClient."""
        super().__init__('add_two_ints_client')

        # Create a client
        # Service name: add_two_ints
        # Service type: example_interfaces/srv/AddTwoInts
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务可用...')

        self.get_logger().info('服务已连接!')

    def send_request(self, a, b):
        """
        Send a request to add two integers.

        Args:
            a: First integer
            b: Second integer

        Returns:
            Future object for the response
        """
        # Create a request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Send the request asynchronously
        self.get_logger().info(f'发送请求: {a} + {b}')
        future = self.cli.call_async(request)

        return future


def main(args=None):
    """Main function to run the service client."""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    node = AddTwoIntsClient()

    # Get numbers from command line or use defaults
    if len(sys.argv) >= 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        a = 3
        b = 5
        node.get_logger().info(f'使用默认值: {a} 和 {b}')

    # Send the request
    future = node.send_request(a, b)

    # Wait for the response
    rclpy.spin_until_future_complete(node, future)

    # Get the result
    try:
        response = future.result()
        node.get_logger().info(f'结果: {response.sum}')
    except Exception as e:
        node.get_logger().error(f'服务调用失败: {e}')

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
