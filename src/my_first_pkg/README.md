# Module 1: 创建第一个ROS 2包和节点

本模块将教你如何创建第一个ROS 2 Python包，并编写简单的节点。

---

## 学习目标

- 理解ROS 2包的结构
- 创建Python包
- 编写简单的ROS 2节点
- 理解节点的生命周期
- 学习两种节点编写方式：函数式和面向对象

---

## 包结构

```
my_first_pkg/
├── my_first_pkg/
│   ├── __init__.py
│   ├── hello_node.py          # 简单的函数式节点
│   └── hello_node_class.py    # 面向对象的节点
├── resource/
│   └── my_first_pkg
├── package.xml                 # 包的元数据
└── setup.py                    # Python包的安装配置
```

---

## 核心概念

### 1. ROS 2节点（Node）

节点是ROS 2中的基本执行单元。每个节点都是一个独立的进程，可以：
- 发布和订阅话题
- 提供和调用服务
- 设置和获取参数
- 执行计算任务

### 2. 包（Package）

包是ROS 2代码的组织单元，包含：
- 源代码
- 配置文件
- 依赖信息
- 构建指令

---

## 文件说明

### package.xml

定义包的元数据和依赖关系：

```xml
<package format="3">
  <name>my_first_pkg</name>
  <version>0.0.1</version>
  <description>My first ROS 2 package</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>  <!-- Python客户端库依赖 -->

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py

配置Python包的安装：

```python
entry_points={
    'console_scripts': [
        'hello = my_first_pkg.hello_node:main',
        'hello_class = my_first_pkg.hello_node_class:main',
    ],
}
```

这里定义了两个可执行文件：
- `hello`: 运行函数式节点
- `hello_class`: 运行面向对象节点

---

## 节点实现

### 方式1: 函数式节点 (hello_node.py)

最简单的节点实现方式：

```python
import rclpy

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)

    # 创建节点
    node = rclpy.create_node('hello_node')

    # 使用节点
    node.get_logger().info('Hello, ROS 2!')

    # 清理
    node.destroy_node()
    rclpy.shutdown()
```

**优点**:
- 代码简洁
- 适合简单的一次性任务

**缺点**:
- 不适合复杂的节点
- 难以维护状态

### 方式2: 面向对象节点 (hello_node_class.py)

使用类来组织节点代码：

```python
from rclpy.node import Node

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node_class')
        self.get_logger().info('Hello from class!')

        # 创建定时器
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Count: {self.counter}')
```

**优点**:
- 代码组织清晰
- 易于维护状态
- 适合复杂节点
- 推荐的方式

---

## 构建和运行

### 1. 构建包

在工作空间根目录：

```bash
cd ~/ros2_ws  # 或你的工作空间路径
colcon build --packages-select my_first_pkg
```

### 2. Source工作空间

```bash
source install/setup.bash
```

### 3. 运行节点

**运行函数式节点**:
```bash
ros2 run my_first_pkg hello
```

输出：
```
[INFO] [hello_node]: Hello, ROS 2! 我是你的第一个节点！
[INFO] [hello_node]: This is my first ROS 2 node!
[INFO] [hello_node]: Node name: hello_node
```

**运行面向对象节点**:
```bash
ros2 run my_first_pkg hello_class
```

输出：
```
[INFO] [hello_node_class]: Hello from HelloNode class!
[INFO] [hello_node_class]: Node initialized: hello_node_class
[INFO] [hello_node_class]: Hello ROS 2! Count: 1
[INFO] [hello_node_class]: Hello ROS 2! Count: 2
...
```

按 `Ctrl+C` 停止节点。

---

## 调试和检查

### 查看运行的节点

```bash
ros2 node list
```

### 查看节点信息

```bash
ros2 node info /hello_node_class
```

输出包括：
- 订阅的话题
- 发布的话题
- 提供的服务
- 使用的动作

---

## 常见问题

### 问题1: 找不到包

**错误**: `Package 'my_first_pkg' not found`

**解决**:
```bash
# 确保已经构建
colcon build --packages-select my_first_pkg

# 确保已经source
source install/setup.bash
```

### 问题2: 导入错误

**错误**: `ModuleNotFoundError: No module named 'my_first_pkg'`

**解决**:
- 检查`__init__.py`文件是否存在
- 重新构建包
- 确保setup.py中的包名正确

### 问题3: 节点无法运行

**解决**:
- 检查Python文件是否有执行权限
- 检查setup.py中的entry_points配置
- 查看错误日志

---

## 练习

1. **修改节点名称**: 将节点名称改为你自己的名字
2. **修改定时器频率**: 将hello_node_class.py中的定时器改为每秒触发一次
3. **添加更多日志**: 在timer_callback中添加更多信息输出
4. **创建新节点**: 创建一个新的节点文件，输出当前时间

### 练习答案提示

修改定时器频率：
```python
# 将2.0改为1.0
self.timer = self.create_timer(1.0, self.timer_callback)
```

输出当前时间：
```python
from datetime import datetime

def timer_callback(self):
    current_time = datetime.now().strftime("%H:%M:%S")
    self.get_logger().info(f'Current time: {current_time}')
```

---

## 下一步

完成本模块后，继续学习：
- [Module 2: 发布者与订阅者](../my_second_pkg/README.md)
- 学习如何在节点之间传递消息

---

## 参考资源

- [ROS 2 Python节点教程](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [rclpy API文档](https://docs.ros2.org/latest/api/rclpy/)
- [ROS 2包创建教程](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
