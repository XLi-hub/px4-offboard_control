# PX4 ROS 2 Offboard Control (Figure-8 Flight)

这是一个基于 **ROS 2** 和 **MicroXRCE-DDS** 协议的 PX4 无人机外部控制（Offboard Control）示例程序。该项目演示了如何通过 C++ 代码控制无人机自动起飞并执行“8字形”飞行轨迹。

## 🚀 功能特性
- **自动起飞**：自动切换至 Offboard 模式并解锁起飞。
- **8字轨迹**：基于参数方程生成平滑的“8字形”位置指令。
- **航向控制 (Yaw)**：动态计算飞行切线方向，使无人机机头始终朝向前进方向。
- **原生通信**：直接使用 `px4_msgs` 与 PX4 uORB 消息对接，无需 MAVROS。

---

## 🛠 环境依赖

在运行本项目之前，请确保你的开发环境已安装以下组件：

1.  **Ubuntu 22.04** (推荐)
2.  **ROS 2 Humble**
3.  **PX4-Autopilot 固件** (v1.14 或更高版本)
4.  **MicroXRCE-DDS Agent** ([安装教程](https://docs.px4.io/main/zh/ros/ros2_comm.html#install-micro-xrce-dds-agent))
5.  **px4_msgs** (必须放在同一个 ROS 2 工作空间内)

---

## 📂 项目结构

建议的工作空间目录结构如下：
```text
px4_ros_ws/
└── src/
    ├── px4_msgs/            # PX4 官方消息定义
    └── px4_offboard/        # 本项目代码
        ├── src/
        │   ├── offboard_control.cpp # 基础起飞代码
        │   └── figure8_node.cpp     # 飞8字代码
        ├── CMakeLists.txt
        └── package.xml
```

---

## 🔨 安装与编译

1. **创建工作空间并克隆代码**:
   ```bash
   mkdir -p ~/px4_ros_ws/src
   cd ~/px4_ros_ws/src
   # 克隆消息定义
   git clone https://github.com/PX4/px4_msgs.git
   # 将本项目代码放入 px4_offboard 文件夹
   ```

2. **编译工作空间**:
   ```bash
   cd ~/px4_ros_ws
   colcon build
   ```

3. **刷新环境变量**:
   ```bash
   source install/setup.bash
   ```

---

## 💻 运行指南

请按照以下顺序打开终端运行：

### 1. 启动 MicroXRCE-DDS Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### 2. 启动 PX4 SITL 仿真
在你的 `PX4-Autopilot` 源码目录下：
```bash
make px4_sitl gazebo-classic
```

### 3. 运行 8 字飞行节点
```bash
source ~/px4_ros_ws/install/setup.bash
ros2 run px4_offboard figure8_node
```

---

## ⚠️ 重要说明：坐标系 (Coordinate System)

本项目使用的是 PX4 原生的 **NED (North-East-Down)** 坐标系：
- **X轴**: 正向为北 (North)。
- **Y轴**: 正向为东 (East)。
- **Z轴**: **负向为上** (Down is positive, so -5.0 means 5m altitude)。

**注意**：这与 ROS 2 常用的 ENU (东北天) 坐标系不同，直接修改代码时请务必留意 Z 轴正负号。

---

## 📈 8字数学公式
轨迹生成采用以下参数方程：
- $x = R \cdot \sin(\theta)$
- $y = R \cdot \sin(2\theta)$
- $\text{Yaw} = \text{atan2}(\dot{y}, \dot{x})$

---

## 📝 许可证
本项目采用 MIT 许可证。仅供学术交流和仿真测试使用，实机飞行请务必在安全环境下进行。

---

### 💡 提示
如果你在使用过程中遇到 `px4_msgs` 找不到头文件的报错，请重新运行 `colcon build` 并确保 `px4_msgs` 文件夹确实存在于 `src` 目录下。
