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
# 行走视角：机器人在地面上真的走
ros2 launch launch_pack spider_walk.launch.py

# 姿态视角：机身钉在原点，只看六条腿怎么动
ros2 launch launch_pack spider_display.launch.py

# 选步态（默认 tripod；ripple / wave 更稳但速度上限更低，见下方说明）
ros2 launch launch_pack spider_walk.launch.py gait_pattern:=ripple
```

RViz 打开后应该看到一台六足机器人站在网格上：

| 部位 | 颜色 | 说明 |
|---|---|---|
| 机身 | 灰 | |
| `coxa`  |  髋关节连杆，水平伸出 |
| `femur` |  大腿，抬膝构型 |
| `tibia` |  小腿，垂直向下踩到地面 |

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

> **速度上限随步态不同**：`tripod` 0.20（默认）/ `ripple` 0.15 / `wave` 0.12 m/s。
> 超过上限不会报错，但步长会被夹住——`leg_calc` 启动时会打印当前步态的上限，命令超限时会打 `WARN`。
>
> **三种步态**（`gait_pattern`，只支持启动时选择）：
> `tripod` 每次 3 条腿摆动（默认，最快）；`ripple` 每次 2 条；`wave` 每次 1 条（最稳）。
> 越稳的步态支撑相占比越大，能支持的最大速度就越低。

### 3. 真实链路（含驱动节点）

```bash
ros2 launch launch_pack spider_minimal.launch.py
```

和上面的区别：**多启动 `robot_driver_node`**，会把 18 路角度打包成 42 字节协议帧。
目前是 `fake-send`（只打印十六进制，不真的发串口），所以不会驱动任何硬件。

**显示模式刻意不带驱动节点**——看姿态的时候不需要产生真数据。

### 4. 手动调试（实物联调第一步）

**绕过数学链**，直接指定 18 路舵机角——发中位、逐路测通道、单腿调试都用它：

```bash
ros2 launch launch_pack spider_manual.launch.py

# 另一个终端：单腿调试——一次设一条腿的 3 路（例：lf 的 coxa / femur / tibia）
ros2 service call /spider/manual_servo/set_angles robot_interfaces/srv/SetServoAngles \
  "{channels: [0, 1, 2], angles: [900, 800, 900]}"
```

启动即发 `default_pose.yaml`（全 900 = 90.0°），之后 1Hz 重发当前值，
driver 晚启动或重启也能收到。**不要和 `leg_calc` 同时运行**——
两个发布者会互相覆盖 `/spider/servo_target`（启动时会互相警告）。

### 5. 跑测试

```bash
# 逐条看结果（日常调试推荐）
./build/leg_calc/test_leg_chain
./build/leg_calc/test_leg_layout
./build/leg_calc/test_servo18_mapper
# ... 共 11 个文件，全部列在 工程现状总结.md 第 8 节

# 官方口径
colcon test --packages-select leg_calc
colcon test-result --all
```

### 6. 启动方式一览

| 命令 | 启动内容 | 用途 |
|---|---|---|
| `ros2 launch launch_pack spider_walk.launch.py` | robot_state_publisher + spider_task + leg_calc + rviz2（`odom` 视角） | **看机器人在地面上走** |
| `ros2 launch launch_pack spider_display.launch.py` | 同上（`spider_base` 视角） | 看每条腿怎么动 |
| `ros2 launch launch_pack spider_minimal.launch.py` | spider_task + leg_calc + robot_driver | 真实链路（协议帧打印，当前 fake-send） |
| `ros2 launch launch_pack spider_manual.launch.py` | manual_servo_node + robot_driver | 手动调试（绕过数学链，直接发舵机角） |
| `ros2 run leg_calc leg_calc_node` | 只有数学层 | 单独调试 / 脚本化验证 |

可用参数：

| 参数 | 适用 | 说明 |
|---|---|---|
| `gait_pattern:=...` | display / walk / minimal | 步态选择：`tripod` / `ripple` / `wave`（默认 `tripod`）；只支持启动时选择 |
| `use_rviz:=false` | display / walk | 只跑数据与 TF、不开 RViz——排查"RViz 里没动"时先这样确认 topic |
| `rviz_config:=spider_walk.rviz` | display | 换 RViz 视角（walk 内部已固定传它） |

不经 launch、直接给节点传参数的写法（例：单独跑数学层用 wave）：

```bash
ros2 run leg_calc leg_calc_node --ros-args -p gait_pattern:=wave
```

---
