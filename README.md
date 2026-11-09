# Spider Upper

六足机器人的**上位机**。ROS2 Humble 工作区，六个包，从高层速度命令到 18 路舵机帧的完整链路。

> **现在能做什么**：`vx / vy / wz` 速度命令 → 步态轨迹 → 六腿 IK → 18 路舵机帧，
> 并且**可以在 RViz 里看见机器人在走**。
>
> 腿长、腿座位置、关节限位都来自机械图纸（`~/Desktop/exist_urdf/hardware`），不是演示数据。
>
> **实机进度（2026-11）**：单腿（3 路舵机）已跑通 `ROS 2 → 串口 → STM32 → 舵机` 全链路，
> 舵机实际姿态与逆解输出一致，`fake_send:=false` 可稳定真发。
> **整机 18 路标定与实机行走尚未完成**——另外 5 条腿未装配，其余通道仍为默认值。
> 驱动层已实现（`serial_port` + 断线自动重连），默认 `fake_send:=true`（只打印帧，不碰设备）。

---

## 仿真演示

▶ **[步态仿真录屏（2026-09-28）](media/simulation-demo.webm)** —— 六条腿在 RViz 中按步态行走

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
ros2 launch launch_pack spider_walk.launch.py
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
默认 `fake_send:=true`（只打印十六进制，不碰设备）；接上硬件后加 `fake_send:=false` 即真发串口
（**设备打不开会直接启动失败**，不静默降级）：

```bash
ros2 launch launch_pack spider_minimal.launch.py fake_send:=false
```

设备名默认是固定名 `/dev/spider_stm32`（需自行安装 udev 规则把设备名固定下来，规则文件尚未入库）。
没装规则时按实际设备名传 `device_name:=/dev/ttyACM0` —— **别写死编号**：
J-Link 插着时它占 `ttyACM0`，下位机就会被挤到 `ttyACM1`，拔掉又变回来。

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
# ... 共 12 个测试文件

# 官方口径
colcon test --packages-select leg_calc
colcon test-result --all
```

### 6. 启动方式一览

| 命令 | 启动内容 | 用途 |
|---|---|---|
| `ros2 launch launch_pack spider_walk.launch.py` | robot_state_publisher + spider_task + leg_calc + rviz2（`odom` 视角） | **看机器人在地面上走** |
| `ros2 launch launch_pack spider_display.launch.py` | 同上（`spider_base` 视角） | 看每条腿怎么动 |
| `ros2 launch launch_pack spider_minimal.launch.py` | spider_task + leg_calc + robot_driver | 真实链路（默认打印帧；`fake_send:=false` 真发） |
| `ros2 launch launch_pack spider_manual.launch.py` | manual_servo_node + robot_driver | 手动调试（绕过数学链，直接发舵机角） |
| `ros2 launch launch_pack spider_leg_ik.launch.py` | leg_ik_node + robot_driver | 单腿 IK 调试（服务调用足端坐标） |
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

## 结构件（3D 打印）

仓库里 `src/spider/model/` 只有 7 个 STL：`body` / `coxa` / `femur` / `tibia` 四个结构件，
加 `mg90s` / `mg996r` / `sg90` 三个舵机参考模型。

**这不是完整的打印件集合**：缺少舵盘、舵机支架等装配件，且部分件与最新图纸尺寸不一致。
完整零件在 FreeCAD 图纸侧维护，未入库。

- 腿 **29 件/条**（整机 ×6）、机身 **6 件**；**建议先打一条腿试装**，验证尺寸与舵机装配后再批量
- 机械图纸与结构设计由机械方向队友完成，本仓库只保留用于 URDF 的模型

---

## 项目状态

| 项 | 状态 |
|---|---|
| ROS 2 工作区（6 包）与 launch 体系 | ✅ 完成 |
| 步态生成（tripod / ripple / wave）与速度上限约束 | ✅ 完成 |
| 六腿 IK（KDL）+ 正解回代校验 + 关节限位 | ✅ 完成 |
| 18 路舵机映射与 42 字节串口帧 | ✅ 完成 |
| 串口驱动（断线重连、dry-run 开关） | ✅ 完成 |
| 单元测试（12 个文件） | ✅ 见下节 |
| RViz 仿真行走 | ✅ 有录屏 |
| **单腿实机验证** | ✅ 2026-11 通过 |
| **整机 18 路标定** | ⬜ 仅单腿 3 路完成 |
| **整机实机行走** | ⬜ 未开始 |
| 地面工况（打滑、触地检测、质心偏移） | ⬜ 未开始 |

> 界定：**单腿实机可用 ≠ 整机可走**。当前只能称为「仿真链路完整 + 单腿实机验证通过」。

---

## 测试记录

`leg_calc` 包下 12 个 gtest 文件，覆盖：

| 测试 | 覆盖内容 |
|---|---|
| `test_leg_chain` | 逆解链路与正解回代一致性 |
| `test_leg_kinematics` | 单腿运动学 |
| `test_leg_layout` | 腿座布局与安装位置 |
| `test_servo18_mapper` | 18 路舵机通道映射 |
| `test_foot_trajectory` | 足端轨迹生成 |
| `test_gait_phase_manager` | 步态相位推进 |
| `test_gait_config_loader` | 步态参数加载 |
| `test_velocity_smoother` | 速度平滑器 |
| `test_odometry_integrator` | 里程计积分 |
| `test_command_watchdog` | 命令超时看门狗 |
| `test_common_types` / `test_gait_types` | 基础类型 |

```bash
colcon test --packages-select leg_calc
colcon test-result --all
```

---

## 已知限制

- **只有单腿接了舵机**：其余 15 路通道的标定值仍为默认，换舵机/换腿需重新标定
- **无触地检测**：足端高度靠运动学假设，真实地面起伏未处理
- **无打滑处理**：步态相位由时间驱动，不是由实际触地反馈驱动
- **目标速度无闭环**：`cmd_vel` 直接映射到步长，尚无里程计速度反馈
- **步态只在启动时选择**：运行中切换需重启节点
- 项目依赖 conda/ROS 混装环境的 `PYTHONPATH` 处理，未做容器化
- 机械模型来自团队图纸，装配公差未在软件侧补偿

---

## 许可

代码部分采用 MIT（见 `LICENSE`）。`src/spider/model/` 下的 STL 与 URDF 参数来自团队机械图纸，
不在本许可范围内，仅用于本项目的仿真与展示。
