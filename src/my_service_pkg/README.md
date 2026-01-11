# Module 3: 服务与客户端

本模块将教你如何使用ROS 2的服务（Service）机制实现请求-响应通信。

---

## 学习目标

- 理解服务通信机制
- 创建服务端（Service Server）
- 创建客户端（Service Client）
- 理解同步和异步服务调用
- 使用example_interfaces服务类型

---

## 包结构

```
my_service_pkg/
├── my_service_pkg/
│   ├── __init__.py
│   ├── add_two_ints_server.py    # 服务端
│   └── add_two_ints_client.py    # 客户端
├── resource/
│   └── my_service_pkg
├── package.xml
└── setup.py
```

---

## 核心概念

### 1. 服务（Service）

服务是ROS 2中的请求-响应通信机制。特点：
- **同步通信**: 客户端等待服务端响应
- **一对一**: 一个请求对应一个响应
- **类型化**: 每个服务有固定的请求和响应类型
- **双向**: 客户端发送请求，服务端返回响应

### 2. 服务 vs 话题

| 特性 | 话题（Topic） | 服务（Service） |
|------|--------------|----------------|
| 通信模式 | 发布-订阅 | 请求-响应 |
| 方向 | 单向 | 双向 |
| 连接 | 多对多 | 一对一 |
| 时序 | 异步 | 同步 |
| 用途 | 持续数据流 | 偶发操作 |

### 3. 服务类型

本例使用 `example_interfaces/srv/AddTwoInts`：

```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

---

## 服务端 (add_two_ints_server.py)

### 关键代码

```python
class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        # 创建服务
        self.srv = self.create_service(
            AddTwoInts,                    # 服务类型
            'add_two_ints',                # 服务名称
            self.add_two_ints_callback     # 回调函数
        )

    def add_two_ints_callback(self, request, response):
        # 处理请求
        response.sum = request.a + request.b
        return response
```

### 服务端API

```python
create_service(
    srv_type,      # 服务类型
    srv_name,      # 服务名称
    callback       # 回调函数
)
```

---

## 客户端 (add_two_ints_client.py)

### 关键代码

```python
class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # 创建客户端
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # 等待服务可用
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务可用...')

    def send_request(self, a, b):
        # 创建请求
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # 异步调用
        future = self.cli.call_async(request)
        return future
```

### 客户端API

```python
create_client(
    srv_type,      # 服务类型
    srv_name       # 服务名称
)
```

---

## 构建和运行

### 1. 构建包

```bash
cd ~/ros2_ws
colcon build --packages-select my_service_pkg
source install/setup.bash
```

### 2. 运行示例

**终端1 - 启动服务端**:
```bash
ros2 run my_service_pkg server
```

输出：
```
[INFO] [add_two_ints_server]: AddTwoInts service server is ready!
```

**终端2 - 启动客户端**:
```bash
# 使用默认值 (3 + 5)
ros2 run my_service_pkg client

# 或指定数字
ros2 run my_service_pkg client 10 20
```

输出：
```
[INFO] [add_two_ints_client]: 服务已连接!
[INFO] [add_two_ints_client]: 发送请求: 10 + 20
[INFO] [add_two_ints_client]: 结果: 30
```

服务端输出：
```
[INFO] [add_two_ints_server]: 收到请求: 10 + 20 = 30
```

---

## 调试和检查

### 查看服务列表

```bash
ros2 service list
```

输出：
```
/add_two_ints
...
```

### 查看服务类型

```bash
ros2 service type /add_two_ints
```

输出：
```
example_interfaces/srv/AddTwoInts
```

### 查看服务接口定义

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

输出：
```
int64 a
int64 b
---
int64 sum
```

### 命令行调用服务

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 7, b: 8}"
```

输出：
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=7, b=8)

response:
example_interfaces.srv.AddTwoInts_Response(sum=15)
```

---

## 常用服务类型

### example_interfaces

```python
from example_interfaces.srv import AddTwoInts, SetBool, Trigger

# AddTwoInts - 加法
request = AddTwoInts.Request()
request.a = 1
request.b = 2

# SetBool - 设置布尔值
request = SetBool.Request()
request.data = True

# Trigger - 触发操作（无参数）
request = Trigger.Request()
```

### std_srvs

```python
from std_srvs.srv import Empty, SetBool, Trigger

# Empty - 空服务（无请求和响应数据）
request = Empty.Request()
```

---

## 同步 vs 异步调用

### 异步调用（推荐）

```python
# 发送请求
future = self.cli.call_async(request)

# 等待响应
rclpy.spin_until_future_complete(node, future)

# 获取结果
response = future.result()
```

### 同步调用（阻塞）

```python
# 同步调用（会阻塞）
response = self.cli.call(request)
```

**注意**: 同步调用会阻塞节点，不推荐在回调函数中使用。

---

## 练习

### 练习1: 修改服务功能

修改服务端，实现乘法而不是加法。

**提示**:
```python
def multiply_callback(self, request, response):
    response.sum = request.a * request.b  # 注意：字段名仍是sum
    return response
```

### 练习2: 添加参数验证

在服务端添加输入验证，拒绝负数。

**提示**:
```python
def add_two_ints_callback(self, request, response):
    if request.a < 0 or request.b < 0:
        self.get_logger().warn('拒绝负数输入')
        response.sum = 0
    else:
        response.sum = request.a + request.b
    return response
```

### 练习3: 使用SetBool服务

创建一个使用`std_srvs/srv/SetBool`的服务端和客户端。

**提示**:
```python
from std_srvs.srv import SetBool

# 服务端
def set_bool_callback(self, request, response):
    response.success = True
    response.message = f'Received: {request.data}'
    return response

# 客户端
request = SetBool.Request()
request.data = True
```

### 练习4: 多次调用

修改客户端，连续调用服务5次。

**提示**:
```python
for i in range(5):
    future = node.send_request(i, i+1)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.get_logger().info(f'Result {i}: {response.sum}')
```

---

## 常见问题

### 问题1: 服务不可用

**错误**: `等待服务可用...`

**解决**:
- 确保服务端正在运行
- 检查服务名称是否匹配
- 使用`ros2 service list`验证

### 问题2: 服务调用超时

**解决**:
```python
# 增加超时时间
if not self.cli.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('服务不可用')
    return
```

### 问题3: 响应为空

**解决**:
- 确保服务端返回了response对象
- 检查服务端是否有异常
- 查看服务端日志

---

## 高级主题

### 1. 服务端多线程

```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

### 2. 服务超时处理

```python
future = self.cli.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)

if future.done():
    response = future.result()
else:
    node.get_logger().error('服务调用超时')
```

### 3. 回调中调用服务

```python
# 在回调中异步调用服务
def timer_callback(self):
    future = self.cli.call_async(request)
    future.add_done_callback(self.response_callback)

def response_callback(self, future):
    response = future.result()
    self.get_logger().info(f'Got response: {response.sum}')
```

---

## 下一步

完成本模块后，继续学习：
- [Module 4: 动作（Action）](../my_action_pkg/README.md)
- 学习长时任务通信机制

---

## 参考资源

- [ROS 2服务教程](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
- [example_interfaces](https://github.com/ros2/example_interfaces)
- [std_srvs](https://github.com/ros2/common_interfaces/tree/jazzy/std_srvs)
