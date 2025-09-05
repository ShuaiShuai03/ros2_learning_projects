import rclpy
from rclpy.node import Node
from std_msgs.msg import String   # 导入标准 String 消息

class TickerNode(Node):
    def __init__(self):
        super().__init__('ticker_node')
        self.i = 0
        self.publisher = self.create_publisher(String, 'tick', 10)  # 创建发布者
        self.timer = self.create_timer(0.5, self.on_timer)          # 每 0.5 秒执行一次

    def on_timer(self):
        self.i += 1
        msg = String()
        msg.data = f'Tick {self.i}'
        self.publisher.publish(msg)              # 发布消息
        self.get_logger().info(f'发布消息: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = TickerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

