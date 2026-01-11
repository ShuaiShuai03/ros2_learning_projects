# Module 2: 发布者与订阅者

本模块将教你如何使用ROS 2的话题（Topic）机制实现节点间的通信。

---

## 学习目标

- 理解发布-订阅模式
- 创建发布者节点（Publisher）
- 创建订阅者节点（Subscriber）
- 理解话题（Topic）和消息（Message）
- 使用定时器（Timer）

---

## 包结构

```
my_second_pkg/
├── my_second_pkg/
│   ├── __init__.py
│   ├── ticker_node.py      # 发布者节点
│   └── listener_node.py    # 订阅者节点
├── resource/
│   └── my_second_pkg
├── package.xml
└── setup.py
```

---

## 核心概念

### 1. 话题（Topic）

话题是ROS 2中节点间通信的命名通道。特点：
- **异步通信**: 发布者和订阅者不需要同时运行
- **多对多**: 一个话题可以有多个发布者和订阅者
- **类型化**: 每个话题有固定的消息类型
- **单向**: 数据从发布者流向订阅者

### 2. 发布-订阅模式

```
┌─────────────┐         /chatter         ┌──────────────┐
│   Ticker    │ ──────────────────────> │   Listener   │
│  (Publisher)│    String messages       │ (Subscriber) │
└─────────────┘                          └──────────────┘
```

### 3. 消息类型

本例使用 `std_msgs/msg/String`：
```python
from std_msgs.msg import String

msg = String()
msg.data = "Hello ROS 2!"
```

---

## 发布者节点 (ticker_node.py)

### 关键代码

```python
class TickerNode(Node):
    def __init__(self):
        super().__init__('ticker_node')

        # 创建发布者
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # 创建定时器（每1秒触发一次）
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.counter = 0

    def timer_callback(self):
        # 创建消息
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'

        # 发布消息
        self.publisher_.publish(msg)

        self.counter += 1
```

### 发布者API

```python
create_publisher(
    msg_type,      # 消息类型
    topic,         # 话题名称
    qos_profile    # 队列大小或QoS配置
)
```

---

## 订阅者节点 (listener_node.py)

### 关键代码

```python
class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')

        # 创建订阅者
        self.subscription = self.create_subscription(
            String,                    # 消息类型
            'chatter',                 # 话题名称
            self.listener_callback,    # 回调函数
            10                         # 队列大小
        )

    def listener_callback(self, msg):
        # 处理接收到的消息
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### 订阅者API

```python
create_subscription(
    msg_type,      # 消息类型
    topic,         # 话题名称
    callback,      # 回调函数
    qos_profile    # 队列大小或QoS配置
)
```

---

## 构建和运行

### 1. 构建包

```bash
cd ~/ros2_ws
colcon build --packages-select my_second_pkg
source install/setup.bash
```

### 2. 运行示例

**终端1 - 启动发布者**:
```bash
ros2 run my_second_pkg ticker
```

输出：
```
[INFO] [ticker_node]: Ticker node started! Publishing to /chatter
[INFO] [ticker_node]: Publishing: "Hello ROS 2! Count: 0"
[INFO] [ticker_node]: Publishing: "Hello ROS 2! Count: 1"
[INFO] [ticker_node]: Publishing: "Hello ROS 2! Count: 2"
...
```

**终端2 - 启动订阅者**:
```bash
ros2 run my_second_pkg listener
```

输出：
```
[INFO] [listener_node]: Listener node started! Subscribed to /chatter
[INFO] [listener_node]: I heard: "Hello ROS 2! Count: 3"
[INFO] [listener_node]: I heard: "Hello ROS 2! Count: 4"
[INFO] [listener_node]: I heard: "Hello ROS 2! Count: 5"
...
```

---

## 调试和检查

### 查看话题列表

```bash
ros2 topic list
```

输出：
```
/chatter
/parameter_events
/rosout
```

### 查看话题信息

```bash
ros2 topic info /chatter
```

输出：
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### 查看话题消息

```bash
ros2 topic echo /chatter
```

### 查看话题频率

```bash
ros2 topic hz /chatter
```

### 手动发布消息

```bash
ros2 topic pub /chatter std_msgs/msg/String "data: 'Manual message'"
```

### 查看消息类型定义

```bash
ros2 interface show std_msgs/msg/String
```

---

## 常用消息类型

### std_msgs

```python
from std_msgs.msg import String, Int32, Float64, Bool

# String
msg = String()
msg.data = "Hello"

# Int32
msg = Int32()
msg.data = 42

# Float64
msg = Float64()
msg.data = 3.14

# Bool
msg = Bool()
msg.data = True
```

### geometry_msgs

```python
from geometry_msgs.msg import Point, Twist

# Point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 3.0

# Twist (速度命令)
twist = Twist()
twist.linear.x = 1.0
twist.angular.z = 0.5
```

---

## QoS (Quality of Service)

QoS配置控制消息传递的可靠性和性能：

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 自定义QoS
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # 可靠传输
    history=HistoryPolicy.KEEP_LAST,         # 保留最近的消息
    depth=10                                  # 队列深度
)

self.publisher_ = self.create_publisher(String, 'chatter', qos_profile)
```

### 常用QoS预设

```python
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

# 传感器数据（最佳努力，可能丢失）
self.create_publisher(String, 'sensor', qos_profile_sensor_data)

# 系统默认（可靠传输）
self.create_publisher(String, 'control', qos_profile_system_default)
```

---

## 练习

### 练习1: 修改发布频率

将ticker_node的发布频率改为2Hz（每0.5秒一次）。

**提示**:
```python
timer_period = 0.5  # seconds
```

### 练习2: 发布数字

创建一个发布Int32消息的节点。

**提示**:
```python
from std_msgs.msg import Int32

self.publisher_ = self.create_publisher(Int32, 'numbers', 10)

msg = Int32()
msg.data = self.counter
```

### 练习3: 多个订阅者

启动多个listener节点，观察它们是否都能接收到消息。

```bash
# 终端1
ros2 run my_second_pkg ticker

# 终端2
ros2 run my_second_pkg listener

# 终端3
ros2 run my_second_pkg listener
```

### 练习4: 消息过滤

修改listener_node，只打印偶数计数的消息。

**提示**:
```python
def listener_callback(self, msg):
    # 提取计数值
    if "Count:" in msg.data:
        count_str = msg.data.split("Count: ")[1]
        count = int(count_str)
        if count % 2 == 0:
            self.get_logger().info(f'Even count: {count}')
```

---

## 常见问题

### 问题1: 订阅者收不到消息

**可能原因**:
- 话题名称不匹配
- 消息类型不匹配
- QoS配置不兼容

**解决**:
```bash
# 检查话题
ros2 topic list
ros2 topic info /chatter

# 检查节点
ros2 node list
ros2 node info /ticker_node
```

### 问题2: 消息延迟

**解决**:
- 增加QoS队列深度
- 使用BEST_EFFORT可靠性策略
- 检查网络和系统负载

### 问题3: 消息丢失

**解决**:
- 使用RELIABLE可靠性策略
- 增加队列深度
- 检查订阅者处理速度

---

## 下一步

完成本模块后，继续学习：
- [Module 3: 服务与客户端](../my_service_pkg/README.md)
- 学习请求-响应通信模式

---

## 参考资源

- [ROS 2话题教程](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 QoS文档](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html)
- [std_msgs消息类型](https://github.com/ros2/common_interfaces/tree/jazzy/std_msgs)
