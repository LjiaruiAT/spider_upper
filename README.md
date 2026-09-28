# Spider Upper

六足机器人的**上位机**。ROS2 Humble 工作区，六个包，从高层速度命令到 18 路舵机帧的完整链路。

> **现在能做什么**：`vx / vy / wz` 速度命令 → 步态轨迹 → 六腿 IK → 18 路舵机帧，
> 并且**可以在 RViz 里看见机器人在走**。
>
> 腿长、腿座位置、关节限位都来自机械图纸（`~/Desktop/exist_urdf/hardware`），不是演示数据。
> 但**舵机还没买**：所有标定值都是 0，真发送也没接（`fake_send` 模式）。
>
> 详细状态、已知问题、下一步计划见 **`工程现状总结.md`**。

---

## 快速开始

### 0. 环境（**不能跳过**）

本机同时装了 Miniconda 和 ROS2 Humble：conda 的 `python3` 是 3.13，ROS 需要 3.10。
不设 `PYTHONPATH` 会撞上 `Importing the numpy C-extensions failed`。

**每个新终端都要先执行这两行：**

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
```

> 嫌烦的长期办法：`conda config --set auto_activate_base false`，然后重开终端。

### 1. 编译

```bash
cd ~/Desktop/spider_upper
colcon build
source install/setup.bash
```

### 2. 在 RViz 里看机器人走 ← **推荐先跑这个**

**终端 A：**

```bash
ros2 launch launch_pack spider_display.launch.py
```

RViz 打开后应该看到一台六足机器人站在网格上：

| 部位 | 颜色 | 说明 |
|---|---|---|
| 机身 | 灰 | |
| `coxa` | **红**（短） | 髋关节连杆，水平伸出 |
| `femur` | **绿**（斜向上） | 大腿，抬膝构型 |
| `tibia` | **蓝**（长） | 小腿，垂直向下踩到地面 |

**终端 B**（同样要先做第 0 步，并 `cd ~/Desktop/spider_upper && source install/setup.bash`）：

```bash
# 前进
ros2 topic pub -r 20 /spider/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'

# 转向
ros2 topic pub -r 20 /spider/cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'

# 斜向走
ros2 topic pub -r 20 /spider/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.1}}'
```

腿会动起来。**按 `Ctrl+C` 停掉发布后，机器人会先平滑减速再停下**
（上游 0.25s 命令超时 + 1s 运动强度斜坡）。

> **速度上限是 0.2 m/s**。超过这个值不会报错，但步长会被夹住——
> `leg_calc` 启动时会打印支持的上限，命令超限时会打 `WARN`。

### 3. 真实链路（含驱动节点）

```bash
ros2 launch launch_pack spider_minimal.launch.py
```

和上面的区别：**多启动 `robot_driver_node`**，会把 18 路角度打包成 42 字节协议帧。
目前是 `fake-send`（只打印十六进制，不真的发串口），所以不会驱动任何硬件。

**显示模式刻意不带驱动节点**——看姿态的时候不需要产生真数据。

### 4. 跑测试

```bash
# 逐条看结果（日常调试推荐）
./build/leg_calc/test_leg_chain
./build/leg_calc/test_leg_layout
./build/leg_calc/test_servo18_mapper
# ... 共 10 个文件，全部列在 工程现状总结.md 第 8 节

# 官方口径
colcon test --packages-select leg_calc
colcon test-result --all
```
---
