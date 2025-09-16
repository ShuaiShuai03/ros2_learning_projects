
# ROS 2 学习教程

欢迎来到本仓库！本项目是一个 **ROS 2 系统化学习教程**，结合 30 天学习计划，从零开始帮助你逐步掌握 ROS 2 的核心概念和开发技能。无论你是初学者、开发者还是准备进入机器人领域的学生，都能通过本教程获得清晰的学习路径和可运行的代码示例。

---

## 📦 仓库结构

```
ros2_learning_projects/
├── src/
│   ├── my_first_pkg/      # 第一个包：Hello ROS 2 节点
│   ├── my_second_pkg/     # 定时器 + 发布/订阅节点
│   ├── my_service_pkg/    # 服务端 + 客户端示例
│   └── my_action_pkg/     # 动作（Fibonacci 示例）
└── .gitignore             # 忽略构建生成文件
```

---

## 🚀 学习模块

* **模块 0 – 环境与工作空间**
  学习如何安装 ROS 2，创建工作空间，并使用 `colcon` 构建和配置开发环境。

* **模块 1 – 节点与包** (`my_first_pkg`)
  理解 ROS 2 包的结构，并编写你的第一个 Python 节点。

* **模块 2 – 发布者与订阅者** (`my_second_pkg`)
  使用定时器构建一个发布者节点，并实现一个订阅者节点来接收消息。

* **模块 3 – 服务与客户端** (`my_service_pkg`)
  编写服务端与客户端，学习请求-响应通信模式（以 `AddTwoInts` 为例）。

* **模块 4 – 动作（Action）** (`my_action_pkg`)
  使用 `example_interfaces/action/Fibonacci` 实现一个动作通信示例，展示如何处理长耗时任务并提供实时反馈与最终结果。

  **运行方法：**
  ```bash
  # 终端 1 – 启动 Action 服务端
  ros2 run my_action_pkg server

  # 终端 2 – 启动 Action 客户端
  ros2 run my_action_pkg client
  ```

  **预期输出（客户端）：**
  ```
  发送请求，order=5
  请求已接受
  收到进度反馈: [0, 1, 1]
  收到进度反馈: [0, 1, 1, 2]
  最终结果: [0, 1, 1, 2, 3]
  ```

* **模块 5 – 高级主题** *(即将推出)*
  包括参数、Launch 启动系统、TF2 坐标变换、URDF 机器人建模、Gazebo 仿真、导航与 SLAM 等。

---

## ⚙️ 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/ShuaiShuai03/ros2_learning_projects.git
cd ros2_learning_projects
```

### 2. 构建工作空间

```bash
colcon build
source install/setup.bash
```

### 3. 运行示例

* **Hello 节点**

```bash
ros2 run my_first_pkg hello
```

输出：

```
[INFO] [hello_node]: Hello, ROS 2! 我是你的第一个节点！
```

* **发布者与订阅者**

```bash
# 终端 1 – 发布者
ros2 run my_second_pkg ticker

# 终端 2 – 订阅者（记得先 source）
ros2 run my_second_pkg listener
```

* **服务与客户端**

```bash
# 终端 1 – 服务端
ros2 run my_service_pkg server

# 终端 2 – 客户端
ros2 run my_service_pkg client
```

输出：

```
[INFO] [add_two_ints_server]: 收到请求: 3 + 5 = 8
[INFO] [add_two_ints_client]: 结果: 8
```

* **动作（Fibonacci 示例）**

```bash
# 终端 1 – 启动 Action 服务端
ros2 run my_action_pkg server

# 终端 2 – 启动 Action 客户端
ros2 run my_action_pkg client
```

输出：

```
发送请求，order=5
请求已接受
收到进度反馈: [0, 1, 1]
收到进度反馈: [0, 1, 1, 2]
最终结果: [0, 1, 1, 2, 3]
```

---

## 📖 学习计划

本教程基于 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) ，参考 30 天学习框架：

1. **基础**：节点、话题、服务、参数
2. **进阶**：TF2、URDF、RViz、Gazebo
3. **实战**：导航、SLAM、完整机器人项目

你可以按照模块顺序学习，也可以根据需要跳读。

---

## 📚 推荐资源

* [ROS 2 官方文档](https://docs.ros.org) – 安装指南、API 参考和教程
* [Awesome ROS 2](https://github.com/fkromer/awesome-ros2) – ROS 2 资源合集
* [ROS Discourse 论坛](https://discourse.ros.org) – 社区问答与资讯
* [ROS 2 示例代码](https://github.com/ros2/examples) – 官方示例

---

## 🤝 贡献

本教程仍在不断完善中，欢迎提出问题、补充内容或提交 PR。如果你也在学习 ROS 2，欢迎 fork 本仓库并分享你的学习心得！

---

## 📜 许可证

MIT License
