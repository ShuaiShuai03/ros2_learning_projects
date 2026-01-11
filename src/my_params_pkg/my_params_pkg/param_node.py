#!/usr/bin/env python3
"""
Node demonstrating parameter usage in ROS 2.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor


class ParamNode(Node):
    """A node that demonstrates parameter usage."""

    def __init__(self):
        """Initialize the ParamNode."""
        super().__init__('param_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter(
            'robot_name',
            'my_robot',
            ParameterDescriptor(description='Name of the robot')
        )

        self.declare_parameter(
            'max_speed',
            1.0,
            ParameterDescriptor(description='Maximum speed in m/s')
        )

        self.declare_parameter(
            'enable_logging',
            True,
            ParameterDescriptor(description='Enable logging')
        )

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_speed = self.get_parameter('max_speed').value
        self.enable_logging = self.get_parameter('enable_logging').value

        # Log initial parameter values
        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
        self.get_logger().info(f'Logging enabled: {self.enable_logging}')

        # Create a timer to periodically check parameters
        self.timer = self.create_timer(2.0, self.timer_callback)

        # Add parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def timer_callback(self):
        """Timer callback that uses parameters."""
        if self.enable_logging:
            self.get_logger().info(
                f'{self.robot_name} running at max speed {self.max_speed} m/s'
            )

    def parameter_callback(self, params):
        """Callback when parameters are changed."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(f'Updated robot_name to: {param.value}')
            elif param.name == 'max_speed':
                if param.value > 0:
                    self.max_speed = param.value
                    self.get_logger().info(f'Updated max_speed to: {param.value}')
                else:
                    self.get_logger().warn('max_speed must be positive!')
                    return SetParametersResult(successful=False)
            elif param.name == 'enable_logging':
                self.enable_logging = param.value
                self.get_logger().info(f'Updated enable_logging to: {param.value}')

        return SetParametersResult(successful=True)


def main(args=None):
    """Main function to run the param node."""
    rclpy.init(args=args)
    node = ParamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
