import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts   # ROS 2 自带的加法服务接口

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('服务端已启动: 等待请求...')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'收到请求: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
