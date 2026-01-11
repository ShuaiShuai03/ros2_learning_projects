# ROS 2 学习教程 - 30天系统化学习项目

欢迎来到本仓库！本项目是一个 **ROS 2 系统化学习教程**，结合 30 天学习计划，从零开始帮助你逐步掌握 ROS 2 的核心概念和开发技能。无论你是初学者、开发者还是准备进入机器人领域的学生，都能通过本教程获得清晰的学习路径和可运行的代码示例。

[![ROS 2](https://img.shields.io/badge/ROS-2%20Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10+-yellow.svg)](https://www.python.org/)

---

## 📦 仓库结构

```
ros2_learning_projects/
├── src/
│   ├── my_first_pkg/       # Module 1: Hello ROS 2 节点
│   ├── my_second_pkg/      # Module 2: 发布者与订阅者
│   ├── my_service_pkg/     # Module 3: 服务端与客户端
│   ├── my_action_pkg/      # Module 4: 动作（Fibonacci 示例）
│   ├── my_params_pkg/      # Module 5: 参数配置
│   └── my_launch_pkg/      # Module 6: Launch 启动系统
├── docs/
│   └── MODULE_0_ENVIRONMENT_SETUP.md  # 环境搭建指南
├── 30_DAY_PLAN.md          # 30天详细学习计划
├── README.md               # 本文件
└── .gitignore              # Git 忽略文件
```

---

## 🚀 快速开始

### 前置要求

- **操作系统**: Ubuntu 22.04 (Humble) 或 Ubuntu 24.04 (Jazzy)
- **ROS 2**: Jazzy Jalisco 或 Humble Hawksbill
- **Python**: 3.10+

### 1. 安装 ROS 2

如果还没有安装 ROS 2，请参考 [环境搭建指南](docs/MODULE_0_ENVIRONMENT_SETUP.md)。

```bash
# Ubuntu 22.04 - ROS 2 Humble
sudo apt install ros-humble-desktop

# Ubuntu 24.04 - ROS 2 Jazzy
sudo apt install ros-jazzy-desktop
```

### 2. 克隆仓库

```bash
git clone https://github.com/YOUR_USERNAME/ros2_learning_projects.git
cd ros2_learning_projects
```

### 3. 安装依赖

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash  # 或 humble

# 安装依赖
rosdep install -i --from-path src --rosdistro jazzy -y
```

### 4. 构建工作空间

```bash
colcon build
source install/setup.bash
```

### 5. 运行示例

#### Module 1: Hello World 节点

```bash
ros2 run my_first_pkg hello
```

输出：
```
[INFO] [hello_node]: Hello, ROS 2! 我是你的第一个节点！
```

#### Module 2: 发布者与订阅者

```bash
# 终端 1 - 发布者
ros2 run my_second_pkg ticker

# 终端 2 - 订阅者
ros2 run my_second_pkg listener
```

#### Module 3: 服务与客户端

```bash
# 终端 1 - 服务端
ros2 run my_service_pkg server

# 终端 2 - 客户端
ros2 run my_service_pkg client 10 20
```

输出：
```
[INFO] [add_two_ints_client]: 结果: 30
```

#### Module 4: 动作（Fibonacci 示例）

```bash
# 终端 1 - Action 服务端
ros2 run my_action_pkg server

# 终端 2 - Action 客户端
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

#### Module 5: 参数配置

```bash
# 使用默认参数
ros2 run my_params_pkg param_node

# 使用配置文件
ros2 run my_params_pkg param_node --ros-args --params-file src/my_params_pkg/config/params.yaml

# 动态修改参数
ros2 param set /param_node max_speed 3.0
```

#### Module 6: Launch 启动系统

```bash
# 启动多个节点
ros2 launch my_launch_pkg multi_node_launch.py
```

---

## 📖 学习模块

### Module 0 – 环境与工作空间
学习如何安装 ROS 2，创建工作空间，并使用 `colcon` 构建和配置开发环境。

📚 [环境搭建指南](docs/MODULE_0_ENVIRONMENT_SETUP.md)

### Module 1 – 节点与包 (`my_first_pkg`)
理解 ROS 2 包的结构，并编写你的第一个 Python 节点。

📚 [详细教程](src/my_first_pkg/README.md)

**学习内容**:
- ROS 2 包结构
- 创建简单节点
- 函数式 vs 面向对象节点
- 日志输出

### Module 2 – 发布者与订阅者 (`my_second_pkg`)
使用定时器构建一个发布者节点，并实现一个订阅者节点来接收消息。

📚 [详细教程](src/my_second_pkg/README.md)

**学习内容**:
- 话题（Topic）通信
- 发布者（Publisher）
- 订阅者（Subscriber）
- 定时器（Timer）
- QoS 配置

### Module 3 – 服务与客户端 (`my_service_pkg`)
编写服务端与客户端，学习请求-响应通信模式（以 `AddTwoInts` 为例）。

📚 [详细教程](src/my_service_pkg/README.md)

**学习内容**:
- 服务（Service）通信
- 服务端（Server）
- 客户端（Client）
- 同步 vs 异步调用

### Module 4 – 动作（Action） (`my_action_pkg`)
使用 `example_interfaces/action/Fibonacci` 实现一个动作通信示例，展示如何处理长耗时任务并提供实时反馈与最终结果。

📚 [详细教程](src/my_action_pkg/README.md)

**学习内容**:
- 动作（Action）通信
- 动作服务端
- 动作客户端
- 反馈（Feedback）机制
- 目标取消

### Module 5 – 参数配置 (`my_params_pkg`)
学习如何声明、使用和动态修改节点参数，以及使用 YAML 配置文件。

📚 [详细教程](src/my_params_pkg/README.md)

**学习内容**:
- 参数声明
- 参数回调
- YAML 配置文件
- 动态参数修改

### Module 6 – Launch 启动系统 (`my_launch_pkg`)
学习如何使用 Launch 文件启动多个节点，传递参数和配置。

📚 [详细教程](src/my_launch_pkg/README.md)

**学习内容**:
- Python Launch 文件
- 启动多个节点
- 参数传递
- 节点重映射

### Module 7-10 – 高级主题 *(规划中)*
- **Module 7**: TF2 坐标变换
- **Module 8**: URDF 机器人建模
- **Module 9**: RViz 可视化
- **Module 10**: Gazebo 仿真

---

## 📅 30天学习计划

本教程提供了一个详细的 30 天学习计划，帮助你系统化地学习 ROS 2。

📚 [查看完整的 30 天学习计划](30_DAY_PLAN.md)

### 学习路径概览

| 周次 | 主题 | 内容 |
|------|------|------|
| 第1周 | 基础入门 | 环境搭建、节点、话题 |
| 第2周 | 通信机制 | 服务、动作、自定义消息 |
| 第3周 | 高级功能 | 参数、Launch、TF2 |
| 第4周 | 机器人建模 | URDF、RViz、Gazebo |
| 第5周+ | 实战项目 | 导航、SLAM、综合项目 |

---

## 🛠️ 常用命令

### 构建相关

```bash
# 构建所有包
colcon build

# 构建特定包
colcon build --packages-select my_first_pkg

# 清理构建
rm -rf build install log

# Source 工作空间
source install/setup.bash
```

### 节点相关

```bash
# 列出所有节点
ros2 node list

# 查看节点信息
ros2 node info /node_name
```

### 话题相关

```bash
# 列出所有话题
ros2 topic list

# 查看话题消息
ros2 topic echo /topic_name

# 发布消息
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# 查看话题频率
ros2 topic hz /topic_name
```

### 服务相关

```bash
# 列出所有服务
ros2 service list

# 调用服务
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### 参数相关

```bash
# 列出参数
ros2 param list

# 获取参数
ros2 param get /node_name parameter_name

# 设置参数
ros2 param set /node_name parameter_name value
```

---

## 🎯 学习建议

1. **循序渐进**: 按照模块顺序学习，不要跳过基础内容
2. **动手实践**: 每个模块都要亲自运行代码并尝试修改
3. **阅读文档**: 经常查阅 [ROS 2 官方文档](https://docs.ros.org)
4. **调试技能**: 学会使用 `ros2 topic echo`、`ros2 node info` 等调试工具
5. **版本控制**: 使用 Git 管理你的代码
6. **社区参与**: 在 [ROS Discourse](https://discourse.ros.org) 提问和分享

---

## 📚 推荐资源

### 官方资源
- [ROS 2 官方文档](https://docs.ros.org/en/jazzy/) - 最权威的学习资源
- [ROS 2 教程](https://docs.ros.org/en/jazzy/Tutorials.html) - 官方教程
- [ROS 2 示例代码](https://github.com/ros2/examples) - 官方示例

### 社区资源
- [Awesome ROS 2](https://github.com/fkromer/awesome-ros2) - ROS 2 资源合集
- [ROS Discourse 论坛](https://discourse.ros.org) - 社区问答
- [The Construct](https://www.theconstructsim.com/) - 在线学习平台

### 视频教程
- [Articulated Robotics](https://www.youtube.com/@ArticulatedRobotics) - 优质 ROS 2 视频教程
- [The Construct YouTube](https://www.youtube.com/@TheConstruct) - ROS 教学视频

---

## 🤝 贡献

本教程仍在不断完善中，欢迎：
- 提出问题（Issues）
- 补充内容（Pull Requests）
- 分享学习心得
- 报告错误

如果你也在学习 ROS 2，欢迎 fork 本仓库并分享你的学习心得！

---

## 📝 常见问题

### Q: 我应该选择 Humble 还是 Jazzy？
A: 如果使用 Ubuntu 22.04，选择 Humble（LTS 版本）；如果使用 Ubuntu 24.04，选择 Jazzy。

### Q: 我可以在 Windows 或 macOS 上学习吗？
A: 可以，但推荐使用 Ubuntu。Windows 可以使用 WSL2，macOS 需要从源码编译。

### Q: 学完这个教程需要多长时间？
A: 如果每天学习 1-2 小时，大约需要 30-45 天完成基础部分。

### Q: 我需要什么编程基础？
A: 需要基本的 Python 编程知识。如果会 C++，可以学习 C++ 版本的 ROS 2。

### Q: 如何获得帮助？
A: 可以在本仓库提 Issue，或在 [ROS Discourse](https://discourse.ros.org) 提问。

---

## 📜 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

---

## 🌟 致谢

感谢 ROS 2 社区的所有贡献者，以及所有为机器人开源事业做出贡献的开发者们！

---

## 📧 联系方式

如有问题或建议，欢迎通过以下方式联系：
- GitHub Issues: [提交问题](https://github.com/YOUR_USERNAME/ros2_learning_projects/issues)
- Email: your.email@example.com

---

**祝你学习愉快！记住，学习 ROS 2 是一个持续的过程，保持好奇心，不断实践，你会成为 ROS 2 专家！** 🚀🤖
