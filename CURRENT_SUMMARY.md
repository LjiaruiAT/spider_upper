# Spider Upper 当前阶段总结

> 用途：给下次继续对话时快速接上当前进度，避免上下文丢失。
> 更新时间：2026-07-25

---

## 1. 当前项目定位

这个工程目前还不是完整的蜘蛛六足机器人上位机，
而是一个**按 ROS2 工程思路逐步搭建的软件骨架**。

当前重点已经从最初的“publisher/subscriber 入门”推进到：
- 学会 ROS2 workspace / package / msg / node / topic
- 建立 `spider_task -> robot_driver` 的最小分层
- 把 `Servo18` 消息映射成下位机 42 字节协议帧
- 给整个工程补齐 6 个目标 package 的外部骨架
- 从参考工程里提取 `leg_calc` 单腿数学核心
- 开始搭“数学结果 -> Servo18 18 路数组”的软件桥
- 给 `spider_task` 加入 `/spider/cmd_vel` 高层控制输入接口骨架
- 把 `leg_calc` 从“单腿演示”推进到“六足静态组织骨架 + 配置驱动默认足端布局”

当前还**没有**进入完整的：
- 步态控制
- 真机上位机部署
- 实际舵机联调
- 完整配置接线
- 实际整机运动控制

---

## 2. 现实硬件约束

### 已有
- 下位机已经有
- 下位机协议已经明确：固定 42 字节帧，18 路角度，单位 0.1°

### 还没有
- 还没有最终上位机板子（计划大二开学后从学长那里买泰山派）
- 舵机还没买，型号还没有最终确认

### 这意味着
当前适合继续做：
- ROS2 软件骨架
- 协议桥接层
- 配置结构设计
- 单腿数学层提取与离线验证
- `Servo18` 软件链路整理

当前**不适合过早深入**：
- 泰山派真机部署细节
- 具体舵机型号绑定参数
- 真正 USB/CDC 到实物的全链路联调

---

## 3. 当前已有 package

```text
spider_upper/
└── src/
    ├── spider/
    ├── robot_interfaces/
    ├── robot_driver/
    ├── leg_calc/
    ├── spider_task/
    └── launch_pack/
```

### 3.1 `spider`
作用：
- 当前作为机器人静态资料 / 配置包

当前已有：
- `config/servo_map.yaml`
- `config/default_pose.yaml`
- `config/leg_params.yaml`
- `model/README.md`

说明：
- 现在配置文件骨架已经有了
- 但这些配置**还没有完全接入运行逻辑**

---

### 3.2 `robot_interfaces`
作用：
- 放全工程共用的消息定义

当前核心消息：
- `msg/Servo18.msg`

内容：
```msg
std_msgs/Header header
int16[18] angle_ddeg
uint8 seq
```

含义：
- `angle_ddeg[18]`：18 路舵机目标角度，单位 0.1°
- `900` 表示 `90.0°`
- `seq` 对应协议中的序号字节

---

### 3.3 `spider_task`
作用：
- 当前作为最小“任务层 / 发布层”

当前节点：

#### `spider_task_node`
作用：
- 周期发布 `/spider/servo_target`
- 当前默认发布 18 路全 `900`
- 同时订阅 `/spider/cmd_vel` 作为高层速度命令输入接口骨架

也就是当前在持续发布：
- 18 路舵机都去 `90.0°`

同时当前已经开始接收：
- `linear.x`：前进 / 后退意图
- `linear.y`：左右平移意图
- `angular.z`：转向意图

说明：
- 当前 `/spider/cmd_vel` 还只是进入 `spider_task` 并被缓存 / 打印
- 还没有正式接到 `leg_calc`
- 这一步的意义是先把高层控制输入接口立起来

#### `servo_listener_node`
作用：
- 订阅 `/spider/servo_target`
- 打印收到的 `Servo18`

说明：
- 这个节点是**学习 / 调试节点**
- 用来理解 subscriber 和验证 topic 通信
- 它不是未来真正的 driver

---

### 3.4 `robot_driver`
作用：
- 当前作为未来下位机桥接层的最小骨架

当前节点：

#### `robot_driver_node`
当前已经做到：
- 订阅 `/spider/servo_target`
- 接收 `Servo18`
- 将其打包成 42 字节协议帧
- 有 `send_frame(...)` 假发送接口
- 可以在 fake-send 模式下打印协议帧和发送结果

这意味着：
- `robot_driver` 已经不再只是“像 subscriber 的练习节点”
- 它已经开始具备真正 driver 的协议层行为和发送层占位结构

同时当前已有：
- `config/driver.yaml`

---

### 3.5 `leg_calc`
作用：
- 当前作为未来数学层 / 单腿计算层的起点

当前已经做到：
- 包骨架已经建立
- 已经从参考工程提取出单腿数学核心 `LegKinematics`
- 已经具备：
  - 单腿 IK（逆运动学）
  - 单腿 FK（正运动学）
  - 速度映射接口
- 已经增加 `Servo18Mapper`，开始搭“关节角结果 -> Servo18 18 路数组”的软件桥
- 已经补上“六足静态组织骨架”，包括：
  - 六足腿序公共类型
  - 六条腿默认足端布局容器
  - 逐腿 IK -> 汇总到 `SpiderJointTargets`
  - 再映射到完整 18 路数组
- 已经开始从 `leg_params.yaml` 读取最小静态布局参数

当前说明：
- 现在 `leg_calc` 还没有正式接入主系统主链
- 但已经不再只是单腿 demo，而是具备六足静态组织能力的数学骨架

---

### 3.6 `launch_pack`
作用：
- 当前作为最小 launch 组织层

当前已有：
- `launch/spider_minimal.launch.py`

作用：
- 一键启动 `spider_task_node`
- 一键启动 `robot_driver_node`

说明：
- 这是当前的最小启动面板
- 更完整的 `display / calibration / walk_test / real` 模式还没有补齐

---

## 4. 当前已完成的通信链路

### ROS2 最小通信闭环
```text
spider_task_node
   ↓ publish
/spider/servo_target   (robot_interfaces/msg/Servo18)
   ↓ subscribe
servo_listener_node
```

这个闭环已经验证通过。

### 初始分层骨架
```text
spider_task_node
   ↓ publish Servo18
/spider/servo_target
   ↓ subscribe
robot_driver_node
   ↓ pack to 42-byte frame
   ↓ fake send / print frame
```

这个分层也已经验证通过。

### 当前数学层软件桥演示
```text
六条腿默认足端目标
   ↓ 六足静态组织
SpiderFootTargets
   ↓ 逐腿 IK
LegKinematics
   ↓ joint angles
SpiderJointTargets
   ↓ Servo18Mapper
18 路 Servo18 兼容数组
```

这个软件桥演示已经从“单腿结果映射”推进到“六足静态组织后再统一映射”，并验证通过。

### 当前高层控制输入骨架
```text
外部输入
   ↓
/spider/cmd_vel   (geometry_msgs/msg/Twist)
   ↓ subscribe
spider_task_node
   ↓ 当前先缓存 / 打印高层命令
未来再逐步接入 leg_calc
```

这说明当前工程已经开始从“直接发最终角度”过渡到“先接收高层控制意图，再逐步下沉到数学层”。

---

## 5. 当前已理解的关键概念

下面这些是本阶段已经重点讲清楚过的内容：

### ROS2 基础
- workspace
- package
- `package.xml`
- `CMakeLists.txt`
- `colcon build`
- `source install/setup.bash`

### 通信模型
- publisher
- subscriber
- topic
- topic 名 + 消息类型匹配
- ROS2 自动匹配通信

### 运行机制
- `main()` 是程序入口
- `rclcpp::init()` 初始化 ROS2
- `rclcpp::spin(node)` 让节点持续运行
- timer 回调
- subscriber 回调

### 层次关系
已经重点区分过：
- `.cpp` 文件：代码写在哪里
- executable：真正运行的程序
- node：ROS2 节点对象
- topic：节点之间的消息通道

### driver 协议层
已经讲过：
- 为什么 `900` 要拆成两个字节
- `int16_t` 是两个字节
- 小端序：低字节在前，高字节在后
- `900 = 0x0384`，所以小端是 `84 03`
- XOR 是简单校验
- 为什么协议必须明确写小端/大端

### 数学层 / 映射层 / 消息层 / 驱动层
已经开始区分：
- 数学层：脚尖目标位置 <-> 关节角
- 映射层：关节角 -> 18 路舵机数组
- 消息层：`Servo18` / topic
- 驱动层：协议组帧 / fake send / 未来真实发送

---

## 6. 当前 42 字节协议帧结构

协议结构：
```text
Byte0   : 0xAA
Byte1   : 0x55
Byte2   : 36
Byte3   : SEQ
Byte4~39: 18 路 int16 小端角度数据
Byte40  : XOR 校验
Byte41  : 0xBB
```

当前在 `robot_driver` 里已经实现：
- `Servo18 -> std::array<uint8_t, 42>`

### 示例
当 18 路全是 `900` 时，数据区会看到大量：
```text
84 03
```
因为：
- `900 = 0x0384`
- 小端编码 = `84 03`

---

## 7. 当前最重要的源码文件

### 配置层
- `src/spider/config/servo_map.yaml`
- `src/spider/config/default_pose.yaml`
- `src/spider/config/leg_params.yaml`
- `src/robot_driver/config/driver.yaml`

### 消息定义
- `src/robot_interfaces/msg/Servo18.msg`

### 发布节点
- `src/spider_task/src/main.cpp`

### 学习用订阅节点
- `src/spider_task/src/servo_listener.cpp`

### driver 节点
- `src/robot_driver/src/main.cpp`

### 数学核心
- `src/leg_calc/include/leg_calc/common_types.hpp`
- `src/leg_calc/include/leg_calc/leg_kinematics.hpp`
- `src/leg_calc/include/leg_calc/servo18_mapper.hpp`
- `src/leg_calc/src/leg_kinematics.cpp`
- `src/leg_calc/src/servo18_mapper.cpp`
- `src/leg_calc/src/main.cpp`

### 启动文件
- `src/launch_pack/launch/spider_minimal.launch.py`

### 编译配置
- `src/robot_interfaces/CMakeLists.txt`
- `src/spider_task/CMakeLists.txt`
- `src/robot_driver/CMakeLists.txt`
- `src/leg_calc/CMakeLists.txt`
- `src/launch_pack/CMakeLists.txt`

### 学习日志
- `LEARNING_LOG.md`

---

## 8. 当前常用命令

### 进入工作区
```bash
cd /home/liujiarui/Desktop/spider_upper
```

### 加载 ROS2
```bash
source /opt/ros/humble/setup.bash
```

### 构建
```bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
colcon build --packages-select robot_interfaces spider_task robot_driver spider leg_calc launch_pack
```

### 加载当前工程
```bash
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
```

### 启动最小主链
```bash
ros2 launch launch_pack spider_minimal.launch.py
```

### 单独启动数学层演示节点
```bash
ros2 run leg_calc leg_calc_node
```

---

## 9. 当前工程离完整六足机器人还差什么

如果按“完整蜘蛛六足机器人”算，当前还差很多，主要缺：

### 配置层真正接线
- `servo_map.yaml` 真正接到 `Servo18Mapper`
- `default_pose.yaml` 真正接到 `spider_task`
- offset / direction / 舵机修正规则

### 通信层后半段
- 真正 USB/CDC 发送
- 与 STM32 实机打通
- 回读和异常处理

### 数学层和控制层
- `leg_calc` 真正接入主链
- 单腿参数与真实蜘蛛腿结构对齐
- 多腿组织
- 步态层
- 站立 / 抬腿 / 前进 / 转向

### 配置层后续扩展
- `servo_offset`
- `direction`
- 更完整的默认姿态参数

### 硬件联调层
- 泰山派部署
- 舵机型号确认
- 标定
- 实机调试

但如果按“软件骨架方向是否正确”算，当前方向是对的，而且已经明显比第一阶段更完整。

---

## 10. 当前最合理的下一步

基于目前的硬件现实约束，下一步最适合做的是：

### 推荐方向
继续做**硬件无关、不会白做**的部分，并把已经存在的配置和数学层真正接线。

### 优先建议
1. 不急着接 USB 真发送
2. 先把 `leg_calc` 的六足静态组织结果真正发布回主链
3. 再让 `spider_task` 和 `leg_calc` 之间形成更明确的“高层意图 -> 六足静态姿态”关系
4. 之后再逐步考虑把更完整的多腿组织和步态层接上

### 比较合理的下一步候选
- 让 `leg_calc_node` 真正发布 `Servo18`，而不只是打印 18 路映射结果
- 继续把 `/spider/cmd_vel` 从“仅缓存 / 打印”推进到“真正影响六足静态组织输出”
- 再往后做“六足静态组织 -> 更完整多腿组织 -> Servo18 消息发布”的系统接线
- 逐步补更多配置参数，例如默认左右镜像、关节偏置和后续真实腿长

---

## 11. 一句话总状态

> 当前 `spider_upper` 已经完成了“6 个 package 的工程骨架 + 自定义消息 + publisher/subscriber + driver 协议组帧 + fake send 发送占位 + 单腿数学核心提取 + Servo18 软件桥骨架 + 六足静态组织骨架与默认足端布局”的阶段性工作，方向正确，基础已经明显超出最初的入门闭环，但还没有进入完整配置接线、整机控制、真实硬件联调和步态阶段。

### 构建
```bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
colcon build --packages-select robot_interfaces spider_task robot_driver
```

### 加载当前工程
```bash
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
```

### 启动发布节点
```bash
ros2 run spider_task spider_task_node
```

### 启动学习订阅节点
```bash
ros2 run spider_task servo_listener_node
```

### 启动 driver 节点
```bash
ros2 run robot_driver robot_driver_node
```

---

## 9. 当前工程离完整六足机器人还差什么

如果按“完整蜘蛛六足机器人”算，当前还差很多，主要缺：

### 通信层后半段
- 真正 USB/CDC 发送
- 与 STM32 实机打通
- 回读和异常处理

### 配置层
- `servo_map`
- `offset`
- `direction`
- 默认姿态参数
- 腿长参数

### 运动学层
- 单腿逆运动学
- 单腿正运动学
- 身体坐标系 / 腿坐标系约定

### 步态层
- 站立
- 抬腿
- 前进
- 转向
- 相位控制

### 硬件联调层
- 泰山派部署
- 舵机型号确认
- 标定
- 实机调试

但如果按“软件骨架方向是否正确”算，当前方向是对的。

---

## 10. 当前最合理的下一步

基于目前的硬件现实约束，下一步最适合做的是：

### 推荐方向
继续做**硬件无关、不会白做**的部分。

### 优先建议
1. 不急着接 USB 真发送
2. 继续让 `robot_driver` 更像真正 driver
3. 开始准备“配置层骨架”或“驱动发送接口骨架”

### 比较合理的下一步候选
- 在 `robot_driver` 里把“组帧”和“发送接口”分层
- 增加一个 `send_frame(...)` 占位接口（先不碰真实设备）
- 或开始搭 `spider` / 配置包骨架，例如：
  - `servo_map.yaml`
  - `default_pose.yaml`
  - `leg_params.yaml`

---

## 11. 一句话总状态

> 当前 `spider_upper` 已经完成了“自定义消息 + publisher/subscriber + driver 骨架 + 42 字节协议组帧”的第一阶段基础工作，已经走上正确的软件分层方向，但还没有进入真实上位机板部署、具体舵机绑定、逆运动学、步态和真机联调阶段。
