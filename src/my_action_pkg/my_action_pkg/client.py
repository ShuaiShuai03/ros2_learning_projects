import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        self.get_logger().info(f'发送请求，order={order}')
        goal = Fibonacci.Goal()
        goal.order = int(order)

        self._client.wait_for_server()
        future = self._client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('请求被拒绝')
            rclpy.shutdown()
            return
        self.get_logger().info('请求已接受')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.on_result)

    def on_feedback(self, feedback_msg):
        seq = list(feedback_msg.feedback.sequence)
        self.get_logger().info(f'收到进度反馈: {seq}')

    def on_result(self, future):
        result = future.result().result
        self.get_logger().info(f'最终结果: {list(result.sequence)}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionClient()
    node.send_goal(5)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
