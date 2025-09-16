import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('收到 Fibonacci 请求！')
        # 反馈与结果字段名都叫 sequence（int32[]）
        feedback = Fibonacci.Feedback()
        feedback.sequence = [0, 1]

        order = max(0, int(goal_handle.request.order or 0))
        if order <= 1:
            # 兼容很小的 order，直接返回当前序列的一部分
            result = Fibonacci.Result()
            result.sequence = feedback.sequence[:order]
            goal_handle.succeed()
            return result

        for i in range(2, order):
            # 计算下一项
            feedback.sequence.append(feedback.sequence[i - 1] + feedback.sequence[i - 2])
            # 发布进度反馈
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f'进度反馈: {feedback.sequence}')
            time.sleep(1.0)  # 模拟耗时

        # 完成
        result = Fibonacci.Result()
        result.sequence = feedback.sequence[:order]
        goal_handle.succeed()
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('收到取消请求')
        return CancelResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
