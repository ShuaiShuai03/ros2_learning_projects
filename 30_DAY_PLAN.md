# ROS 2 30天学习计划

本文档提供了一个系统化的30天ROS 2学习路径，帮助你从零基础逐步掌握ROS 2的核心概念和实战技能。

---

## 第一周：基础入门（Day 1-7）

### Day 1-2: 环境搭建与基础概念
- **目标**: 安装ROS 2，理解ROS 2的基本架构
- **内容**:
  - 安装ROS 2 Jazzy/Humble
  - 配置开发环境
  - 理解节点（Node）、话题（Topic）、服务（Service）、动作（Action）的概念
  - 学习ROS 2命令行工具（ros2 run, ros2 topic, ros2 node等）
- **实践**: 运行turtlesim示例，观察节点通信

### Day 3-4: 创建第一个包和节点
- **目标**: 创建工作空间，编写第一个ROS 2节点
- **内容**:
  - 使用colcon构建系统
  - 创建Python/C++包
  - 编写Hello World节点
  - 理解package.xml和setup.py/CMakeLists.txt
- **实践**: `my_first_pkg` - 创建并运行简单的节点
- **参考**: `src/my_first_pkg/`

### Day 5-6: 发布者与订阅者
- **目标**: 掌握话题通信机制
- **内容**:
  - 理解发布-订阅模式
  - 创建发布者节点（Publisher）
  - 创建订阅者节点（Subscriber）
  - 使用标准消息类型（std_msgs）
  - 使用定时器（Timer）
- **实践**: `my_second_pkg` - 实现计数器发布者和监听者
- **参考**: `src/my_second_pkg/`

### Day 7: 自定义消息类型
- **目标**: 创建和使用自定义消息
- **内容**:
  - 定义.msg文件
  - 配置包依赖
  - 在节点中使用自定义消息
- **实践**: 创建自定义消息并在发布者/订阅者中使用

---

## 第二周：通信机制深入（Day 8-14）

### Day 8-9: 服务（Service）
- **目标**: 掌握请求-响应通信模式
- **内容**:
  - 理解服务通信机制
  - 创建服务端（Service Server）
  - 创建客户端（Service Client）
  - 使用标准服务类型
- **实践**: `my_service_pkg` - 实现加法服务
- **参考**: `src/my_service_pkg/`

### Day 10-11: 自定义服务类型
- **目标**: 创建和使用自定义服务
- **内容**:
  - 定义.srv文件
  - 同步和异步服务调用
  - 服务超时处理
- **实践**: 创建自定义服务（如字符串处理服务）

### Day 12-13: 动作（Action）
- **目标**: 掌握长时任务通信机制
- **内容**:
  - 理解动作通信机制
  - 创建动作服务端（Action Server）
  - 创建动作客户端（Action Client）
  - 处理反馈（Feedback）和结果（Result）
- **实践**: `my_action_pkg` - 实现Fibonacci动作
- **参考**: `src/my_action_pkg/`

### Day 14: 复习与综合练习
- **目标**: 巩固第一、二周知识
- **实践**: 创建一个综合项目，使用话题、服务和动作

---

## 第三周：高级功能（Day 15-21）

### Day 15-16: 参数（Parameters）
- **目标**: 掌握节点参数配置
- **内容**:
  - 声明和使用参数
  - 参数回调函数
  - 使用YAML配置文件
  - 动态参数修改
- **实践**: `my_params_pkg` - 创建可配置的节点
- **参考**: `src/my_params_pkg/`

### Day 17-18: Launch启动系统
- **目标**: 掌握多节点启动管理
- **内容**:
  - 编写Python Launch文件
  - 启动多个节点
  - 传递参数和重映射
  - 包含其他Launch文件
- **实践**: `my_launch_pkg` - 创建复杂的启动配置
- **参考**: `src/my_launch_pkg/`

### Day 19-20: TF2坐标变换
- **目标**: 理解和使用坐标系统
- **内容**:
  - TF2基本概念
  - 发布静态和动态变换
  - 监听坐标变换
  - 使用tf2_ros
- **实践**: `my_tf2_pkg` - 实现坐标变换示例
- **参考**: `src/my_tf2_pkg/`

### Day 21: 时间与时钟
- **目标**: 掌握ROS 2时间系统
- **内容**:
  - ROS时间 vs 系统时间
  - 使用仿真时间
  - 时间同步

---

## 第四周：机器人建模与仿真（Day 22-28）

### Day 22-23: URDF机器人建模
- **目标**: 创建机器人模型
- **内容**:
  - URDF基础语法
  - 定义link和joint
  - 添加视觉和碰撞属性
  - 使用Xacro简化URDF
- **实践**: `my_robot_description` - 创建简单机器人模型
- **参考**: `src/my_robot_description/`

### Day 24-25: RViz可视化
- **目标**: 使用RViz进行可视化
- **内容**:
  - RViz基本操作
  - 显示机器人模型
  - 可视化传感器数据
  - 配置RViz显示
- **实践**: 在RViz中显示自定义机器人

### Day 26-27: Gazebo仿真
- **目标**: 在Gazebo中仿真机器人
- **内容**:
  - Gazebo基础
  - 加载机器人模型到Gazebo
  - 添加传感器插件
  - 控制仿真机器人
- **实践**: `my_gazebo_pkg` - 在Gazebo中仿真机器人
- **参考**: `src/my_gazebo_pkg/`

### Day 28: 传感器数据处理
- **目标**: 处理激光雷达和相机数据
- **内容**:
  - 订阅sensor_msgs
  - 处理LaserScan数据
  - 处理Image数据
  - 使用cv_bridge

---

## 第五周及以后：实战项目（Day 29-30+）

### Day 29: 导航系统入门
- **目标**: 了解Nav2导航栈
- **内容**:
  - Nav2架构概述
  - 地图创建（SLAM）
  - 路径规划
  - 避障
- **实践**: 使用Nav2进行简单导航

### Day 30: 综合项目
- **目标**: 整合所学知识
- **实践**: 创建一个完整的机器人应用
  - 使用自定义消息和服务
  - 实现状态机
  - 集成传感器
  - 实现自主导航

### 进阶方向（Day 30+）
- **SLAM建图**: Cartographer, SLAM Toolbox
- **机械臂控制**: MoveIt 2
- **计算机视觉**: OpenCV, PCL点云处理
- **机器学习集成**: TensorFlow, PyTorch
- **多机器人系统**: 命名空间，多机器人协调
- **实时性能**: DDS配置，QoS策略优化

---

## 学习建议

1. **每天实践**: 理论学习后必须动手编写代码
2. **阅读文档**: 经常查阅[ROS 2官方文档](https://docs.ros.org)
3. **调试技能**: 学会使用ros2 topic echo, ros2 node info等调试工具
4. **版本控制**: 使用Git管理你的代码
5. **社区参与**: 在ROS Discourse论坛提问和分享
6. **循序渐进**: 不要跳过基础内容，扎实掌握每个概念

---

## 推荐资源

- [ROS 2官方文档](https://docs.ros.org/en/jazzy/)
- [ROS 2 Tutorials](https://docs.ros.org/en/jazzy/Tutorials.html)
- [The Construct](https://www.theconstructsim.com/) - 在线ROS学习平台
- [Articulated Robotics YouTube频道](https://www.youtube.com/@ArticulatedRobotics)
- [ROS 2示例代码](https://github.com/ros2/examples)

---

祝你学习愉快！记住，学习ROS 2是一个持续的过程，30天只是开始。保持好奇心，不断实践，你会成为ROS 2专家！
