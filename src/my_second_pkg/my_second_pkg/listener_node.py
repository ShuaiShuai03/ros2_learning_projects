import rclpy
from rclpy.node import Node
from std_msgs.msg import String		# 接收 String 消息

class ListenerNode(Node):
	def __init__(self):
		super().__init__('listener_node')
		self.subscription = self.create_subscription(
			String,		# 消息类型
			'tick',		# 订阅的话题名
			self.callback,	# 收到消息时的回调函数
			10		# 队列长度
		)
		self.subscription	# 防止变量被垃圾收回

	def callback(self, msg):
		self.get_logger().info(f'收到消息："{msg.data}"')

def main(args=None):
	rclpy.init(args=args)
	node = ListenerNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
