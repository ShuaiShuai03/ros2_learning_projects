# Module 0: 环境搭建与工作空间配置

本模块将指导你完成ROS 2的安装和开发环境配置。

---

## 系统要求

- **操作系统**: Ubuntu 22.04 (推荐) 或 Ubuntu 24.04
- **ROS 2版本**: Jazzy Jalisco (Ubuntu 24.04) 或 Humble Hawksbill (Ubuntu 22.04)
- **Python**: 3.10+
- **磁盘空间**: 至少10GB可用空间

---

## 1. 安装ROS 2

### Ubuntu 22.04 - ROS 2 Humble

```bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS 2 apt仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# 安装开发工具
sudo apt install ros-dev-tools
```

### Ubuntu 24.04 - ROS 2 Jazzy

```bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS 2 apt仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS 2
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

# 安装开发工具
sudo apt install ros-dev-tools
```

---

## 2. 配置环境

### 设置ROS 2环境变量

每次打开新终端时需要source ROS 2的setup文件：

```bash
# For Humble
source /opt/ros/humble/setup.bash

# For Jazzy
source /opt/ros/jazzy/setup.bash
```

### 自动配置（推荐）

将source命令添加到`.bashrc`文件中：

```bash
# For Humble
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# For Jazzy
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 重新加载配置
source ~/.bashrc
```

---

## 3. 验证安装

### 测试ROS 2命令

```bash
# 检查ROS 2版本
ros2 --version

# 列出可用的ROS 2命令
ros2 --help
```

### 运行示例程序

打开两个终端：

**终端1 - 运行talker节点：**
```bash
ros2 run demo_nodes_cpp talker
```

**终端2 - 运行listener节点：**
```bash
ros2 run demo_nodes_cpp listener
```

如果能看到talker发送消息，listener接收消息，说明安装成功！

---

## 4. 创建工作空间

### 创建工作空间目录

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 构建工作空间

```bash
# 安装colcon构建工具（如果还没安装）
sudo apt install python3-colcon-common-extensions

# 构建工作空间
colcon build

# Source工作空间
source install/setup.bash
```

### 将工作空间添加到.bashrc（可选）

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 5. 安装常用工具

### 安装开发工具

```bash
# Python依赖
sudo apt install python3-pip python3-pytest-cov python3-flake8

# C++开发工具
sudo apt install build-essential cmake git

# ROS 2工具
sudo apt install ros-$ROS_DISTRO-rqt*
sudo apt install ros-$ROS_DISTRO-rviz2
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```

### 安装常用ROS 2包

```bash
# 示例接口
sudo apt install ros-$ROS_DISTRO-example-interfaces

# TF2工具
sudo apt install ros-$ROS_DISTRO-tf2-tools ros-$ROS_DISTRO-tf-transformations

# 机器人状态发布器
sudo apt install ros-$ROS_DISTRO-robot-state-publisher

# Joint State Publisher
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui
```

---

## 6. 配置IDE（可选）

### VS Code配置

1. 安装VS Code
2. 安装扩展：
   - Python
   - C/C++
   - ROS
   - CMake Tools

3. 创建`.vscode/c_cpp_properties.json`：

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

---

## 7. 常用ROS 2命令

### 节点相关

```bash
# 列出所有运行的节点
ros2 node list

# 查看节点信息
ros2 node info /node_name
```

### 话题相关

```bash
# 列出所有话题
ros2 topic list

# 查看话题消息类型
ros2 topic info /topic_name

# 查看话题消息内容
ros2 topic echo /topic_name

# 发布消息到话题
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello ROS 2'"
```

### 服务相关

```bash
# 列出所有服务
ros2 service list

# 查看服务类型
ros2 service type /service_name

# 调用服务
ros2 service call /service_name example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

### 参数相关

```bash
# 列出节点的所有参数
ros2 param list

# 获取参数值
ros2 param get /node_name parameter_name

# 设置参数值
ros2 param set /node_name parameter_name value
```

### 包相关

```bash
# 列出所有包
ros2 pkg list

# 查看包的可执行文件
ros2 pkg executables package_name

# 查看包的路径
ros2 pkg prefix package_name
```

---

## 8. 故障排除

### 问题1: 找不到ros2命令

**解决方案**: 确保已经source了ROS 2的setup文件
```bash
source /opt/ros/humble/setup.bash  # 或 jazzy
```

### 问题2: colcon build失败

**解决方案**:
- 检查是否安装了所有依赖：`rosdep install -i --from-path src --rosdistro humble -y`
- 清理构建文件：`rm -rf build install log`

### 问题3: 节点无法通信

**解决方案**:
- 检查ROS_DOMAIN_ID是否一致
- 检查防火墙设置
- 使用`ros2 doctor`诊断问题

---

## 9. 下一步

环境配置完成后，你可以：

1. 学习[Module 1: 创建第一个包和节点](../src/my_first_pkg/README.md)
2. 阅读[30天学习计划](../30_DAY_PLAN.md)
3. 浏览[ROS 2官方教程](https://docs.ros.org/en/jazzy/Tutorials.html)

---

## 参考资源

- [ROS 2安装指南](https://docs.ros.org/en/jazzy/Installation.html)
- [Colcon文档](https://colcon.readthedocs.io/)
- [ROS 2命令行工具](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html)
