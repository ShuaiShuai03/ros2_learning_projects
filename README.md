# ROS2 Learning Projects

本仓库记录了我在 **30 天 ROS 2 学习计划** 中完成的练习与项目。  
从零开始，通过一个个小项目逐步掌握 ROS 2 的核心概念与开发技能。

---

## 📂 项目结构
```
ros2_learning_projects/
├── src/
│   ├── my_first_pkg/      # 第一个包：Hello ROS 2 节点
│   └── my_second_pkg/     # 第二个包：定时器 + 发布/订阅节点
└── .gitignore             # 忽略构建生成文件
```

---

## 🚀 学习目标
- [x] Day 0: 环境搭建与验证
- [x] Day 1: 工作空间、包、节点
- [x] Day 2: 定时器与日志
- [x] Day 3: 话题通信（Publisher + Subscriber）
- [ ] Day 4+: 更多 ROS 2 功能（Service, Action, Launch, TF2, URDF, Gazebo, SLAM...）

---

## ⚙️ 使用方法

### 1. 克隆仓库
```bash
git clone https://github.com/ShuaiShuai03/ros2_learning_projects.git
cd ros2_learning_projects
```

### 2. 构建项目
```bash
colcon build
source install/setup.bash
```

### 3. 运行示例节点

#### my_first_pkg
```bash
ros2 run my_first_pkg hello
```
输出：
```
[INFO] [hello_node]: Hello, ROS 2! 我是你的第一个节点！
```

#### my_second_pkg - 发布者
```bash
ros2 run my_second_pkg ticker
```
输出：
```
[INFO] [ticker_node]: 发布消息: "Tick 1"
[INFO] [ticker_node]: 发布消息: "Tick 2"
...
```

#### my_second_pkg - 订阅者
新开一个终端（记得先 `source install/setup.bash`）：
```bash
ros2 run my_second_pkg listener
```
输出：
```
[INFO] [listener_node]: 收到消息: "Tick 1"
[INFO] [listener_node]: 收到消息: "Tick 2"
...
```

---

## 📖 学习计划
本项目基于 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) 学习，目标是在 30 天内完成以下内容：
1. 基础：节点、话题、服务、参数
2. 进阶：TF2、URDF、RViz、Gazebo
3. 实战：导航、SLAM、完整机器人项目

我会持续更新，把每天的学习内容整理成代码和文档上传到这里。

---

## 🤝 贡献
这是一个学习性质的仓库，欢迎任何建议或改进意见。  
如果你也在学习 ROS 2，可以 fork 本仓库，一起交流学习！

---

## 📜 License
MIT License
