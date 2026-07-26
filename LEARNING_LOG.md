# Spider Upper 学习日志

> 这份日志用于记录这个工程是怎样一步一步搭起来的。  
> 每次只记一个小步骤，方便回顾“当时加了什么、学到了什么、下一步做什么”。

---

## 常用命令

### 1. 进入工作区
```bash
cd /home/liujiarui/Desktop/spider_upper
```

### 2. 加载 ROS2 环境
```bash
source /opt/ros/humble/setup.bash
```

### 3. 构建当前工程
```bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
colcon build --packages-select robot_interfaces spider_task robot_driver spider leg_calc launch_pack
```

### 4. 加载当前工程环境
```bash
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
```

### 5. 启动发布节点
```bash
ros2 run spider_task spider_task_node
```

### 6. 启动订阅节点
```bash
ros2 run spider_task servo_listener_node
```

### 7. 启动最小 driver 节点
```bash
ros2 run robot_driver robot_driver_node
```

### 8. 启动数学层演示节点
```bash
ros2 run leg_calc leg_calc_node
```

### 9. 一个终端发，一个终端收
终端 A：
```bash
cd /home/liujiarui/Desktop/spider_upper
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
ros2 run spider_task spider_task_node
```

终端 B：
```bash
cd /home/liujiarui/Desktop/spider_upper
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
ros2 run spider_task servo_listener_node
```

### 10. 一个终端发，一个终端给 driver
终端 A：
```bash
cd /home/liujiarui/Desktop/spider_upper
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
ros2 run spider_task spider_task_node
```

终端 B：
```bash
cd /home/liujiarui/Desktop/spider_upper
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
ros2 run robot_driver robot_driver_node
```

---

## Launch 启动方式

### 当前最小主链一键启动
启动命令：
```bash
ros2 launch launch_pack spider_minimal.launch.py
```

### 先配好环境再启动
完整步骤：
```bash
cd /home/liujiarui/Desktop/spider_upper
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
ros2 launch launch_pack spider_minimal.launch.py
```

### 这个命令会启动什么
当前会一起启动：
- `spider_task_node`
- `robot_driver_node`

也就是这条最小主链：
```text
spider_task_node
   ↓ publish Servo18
/spider/servo_target
   ↓ subscribe
robot_driver_node
   ↓ pack_frame(...)
42-byte frame
   ↓ send_frame(...)
fake-send / future real device
```

### 当前最小主链启动后，如何测试高层控制输入
另开一个终端，先配同样的环境，再发送一条测试命令：
```bash
cd /home/liujiarui/Desktop/spider_upper
source /opt/ros/humble/setup.bash
export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/usr/lib/python3/dist-packages
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/liujiarui/Desktop/spider_upper/install/robot_interfaces/local/lib/python3.10/dist-packages
ros2 topic pub /spider/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}' -1
```

预期现象：
- `spider_task_node` 会打印收到的 `/spider/cmd_vel`
- `robot_driver_node` 仍然继续处理当前最小 `Servo18` 主链

### 什么时候用这个启动方式
适合：
- 想直接跑当前最小系统
- 不想手动开两个终端
- 想验证 `spider_task -> robot_driver` 主链是否正常

### 和 `ros2 run` 的区别
- `ros2 launch ...`：一次启动一组节点
- `ros2 run ...`：一次只启动一个节点

所以：
- `launch` 更像“整套系统启动入口”
- `run` 更像“单节点调试入口”

---

## 当前工程架构

### 1. 当前 6 个 package
```text
spider_upper/
└── src/
    ├── spider/              # 配置和模型资料包
    ├── robot_interfaces/    # 公共消息定义
    ├── robot_driver/        # 下位机协议桥和发送层骨架
    ├── leg_calc/            # 单腿数学核心和 Servo18 软件桥骨架
    ├── spider_task/         # 当前最小任务层 / 发布层
    └── launch_pack/         # launch 启动组织层
```

### 2. 当前主数据流
```text
spider_task_node
   ↓ publish Servo18
/spider/servo_target
   ↓ subscribe
robot_driver_node
   ↓ pack_frame(...)
42-byte frame
   ↓ send_frame(...)
fake-send / future real device
```

### 3. 当前数学层演示流
```text
单腿脚尖目标点
   ↓ IK
LegKinematics
   ↓ 关节角
SpiderJointTargets
   ↓ Servo18Mapper
18 路 Servo18 兼容数组
```

### 4. 当前最重要的四层概念
- 数学层：IK / FK / 单腿关节计算
- 映射层：关节角 -> 18 路 Servo18 数组
- 消息层：`Servo18` 和 `/spider/servo_target`
- 驱动层：42 字节协议组帧、fake send、未来真实发送

### 5. 当前高层控制输入接口
当前已经增加：
- `/spider/cmd_vel`
- 消息类型：`geometry_msgs/msg/Twist`

当前约定先使用：
- `linear.x`：前进 / 后退意图
- `linear.y`：左右平移意图
- `angular.z`：转向意图

说明：
- 这条命令当前先进入 `spider_task_node`
- 现在只做缓存和打印
- 以后再逐步接到 `leg_calc`

### 6. 当前最小启动方式
#### 方式 A：一键启动当前主链
```bash
ros2 launch launch_pack spider_minimal.launch.py
```
它会启动：
- `spider_task_node`
- `robot_driver_node`

#### 方式 B：分终端单独启动
- 用 `ros2 run spider_task spider_task_node` 启动发布层
- 用 `ros2 run robot_driver robot_driver_node` 启动 driver 层
- 用 `ros2 run spider_task servo_listener_node` 启动学习型订阅节点

#### 方式 C：单独验证数学层
```bash
ros2 run leg_calc leg_calc_node
```
它当前会演示：
- 单腿 IK
- FK 回代
- 映射到 18 路数组

---

## 第 1 条记录

### 时间
2026-07-23

### 本次新增 / 修改的文件
- `src/robot_interfaces/package.xml`
- `src/robot_interfaces/CMakeLists.txt`
- `src/robot_interfaces/msg/Servo18.msg`
- `src/spider_task/package.xml`
- `src/spider_task/CMakeLists.txt`
- `src/spider_task/src/main.cpp`
- `LEARNING_LOG.md`

### 本次加了什么
- 建立了第一个消息包 `robot_interfaces`
- 定义了第一个自定义消息 `Servo18.msg`
- 建立了第一个功能包 `spider_task`
- 写了一个最小发布节点，让它周期发布 `/spider/servo_target`
- 新建了这份学习日志，用来持续记录工程推进过程

### 这一步主要学到什么
- ROS2 工程通常放在 workspace 里，通过 `src/` 管理源码包
- `robot_interfaces` 适合单独存放“多个节点共用的消息定义”
- `.msg` 文件本质上是在定义节点之间通信的数据格式
- `spider_task` 这种包更像“具体干活的节点”
- `colcon build` 会生成 `build/`、`install/`、`log/`，其中 `install/` 是后续 `source` 和运行时真正会用到的结果目录

### 当前状态
- `robot_interfaces` 和 `spider_task` 已经建立
- `Servo18.msg` 已经生成成功
- `spider_task_node` 已经可以运行并发布消息
- 当前环境里存在一些 Python / ROS2 CLI 解析自定义消息的环境问题，后续需要继续整理，但不影响我们继续按小步骤学习

### 下一步
- 先不要急着继续加大模块
- 下一小步优先把一个 ROS2 基础概念讲透，例如：
  - 为什么 `source install/setup.bash` 之后就能找到自己的包
  - 或者什么是 publisher / subscriber 的配合关系
- 等你理解后，再决定是补一个最小 subscriber，还是继续整理工程结构

---

## 第 2 条记录

### 时间
2026-07-23 16:47:19

### 本次新增 / 修改的文件
- `src/spider_task/src/main.cpp`
- `src/spider_task/src/servo_listener.cpp`
- `src/spider_task/CMakeLists.txt`
- `LEARNING_LOG.md`

### 本次加了什么
- 给 `spider_task/src/main.cpp` 增加了学习型注释
- 新增了一个最小订阅节点 `servo_listener_node`
- 让它订阅 `/spider/servo_target` 并打印收到的 `Servo18` 消息
- 在 `CMakeLists.txt` 里把新的订阅节点加入编译和安装
- 把当前工程的 build 和启动命令补充到了日志顶部，方便反复使用

### 这一步主要学到什么
- ROS2 里 `publisher` 负责发消息，`subscriber` 负责收消息
- `topic` 是两者之间共享的消息通道名字
- 同一个 topic 可以被多个节点订阅，发布者不需要知道接收者是谁
- `CMakeLists.txt` 里不仅要编译发布节点，也要把新的订阅节点加入 `add_executable(...)`、依赖和 `install(...)`
- 一个工程如果要反复学习和实验，把常用命令记在日志里会更顺手

### 当前状态
- `spider_task_node` 会周期发布 `/spider/servo_target`
- `servo_listener_node` 已经能收到并打印 18 路 `900` 的目标值
- 这说明当前工程已经完成了一个最小 ROS2 通信闭环：一个节点发，一个节点收

### 下一步
- 先别急着接 USB 或下位机
- 下一小步更适合继续巩固 ROS2 基础，例如：
  - 逐行理解 `create_publisher(...)` 和 `create_subscription(...)`
  - 或者把 `900` 改成参数，让节点可发布不同测试姿态

---

## 第 3 条记录

### 时间
2026-07-23 18:59:57

### 本次新增 / 修改的文件
- `LEARNING_LOG.md`

### 本次加了什么
- 没有新增功能代码，这一步主要是在现有工程基础上做阶段性总结
- 把最近几轮真正搞清楚的 ROS2 基础概念整理成第三次学习记录

### 这一步主要学到什么
- 一个 package 里可以有多个可执行程序，所以会出现多个 `main()`
- `.cpp` 文件、可执行程序、ROS2 节点不是同一个层次的东西
- 不是“文件和文件”在通过 topic 通信，而是文件里定义出来、运行起来的节点在通信
- publisher 和 subscriber 之所以能连上，是因为 topic 名一致、消息类型一致，ROS2 底层会自动匹配
- `rclcpp::spin(node)` 的作用是让节点持续运行，并处理定时器和订阅回调等事件
- 节点写成类，是因为它通常要长期持有 publisher、subscriber、timer、状态变量和回调函数

### 当前状态
- 当前工程已经完成了一个最小 ROS2 通信闭环
- 你现在已经能区分：源码文件、可执行程序、节点、topic 这几个层次
- 当前工程重点仍然是 ROS2 基础学习，不是完整蜘蛛上位机功能

### 下一步
- 可以继续巩固 ROS2 基础，比如继续逐行理解现有节点代码
- 也可以开始往下一层走，慢慢搭一个最小 `robot_driver` 骨架，但第一版只做订阅和打印

---

## 第 4 条记录

### 时间
2026-07-23 19:16:06

### 本次新增 / 修改的文件
- `src/robot_driver/package.xml`
- `src/robot_driver/CMakeLists.txt`
- `src/robot_driver/src/main.cpp`
- `LEARNING_LOG.md`

### 本次加了什么
- 新建了 `robot_driver` package
- 写了第一版最小 `robot_driver_node`
- 让它订阅 `/spider/servo_target`，收到 `Servo18` 后打印 18 路角度
- 更新了日志顶部常用命令，把 `robot_driver` 的 build 和启动命令也补进去了

### 这一步主要学到什么
- `servo_listener_node` 是学习/调试 subscriber 的节点，而 `robot_driver_node` 是系统分层里真正下一层的骨架
- 即使第一版都只是“订阅并打印”，工程角色也不一样：一个偏观察，一个偏桥接层起点
- 新建一个 ROS2 package 时，需要同时补 `package.xml`、`CMakeLists.txt` 和最小 `main.cpp`
- 现在工程开始从“会发会收”走向“开始分层”

### 当前状态
- `spider_task_node` 可以持续发布 `/spider/servo_target`
- `servo_listener_node` 仍然可用，继续作为调试/学习节点
- `robot_driver_node` 已经能正常收到并打印 `spider_task_node` 发出的 18 路 `900` 目标值
- 当前工程已经开始具备 `spider_task -> robot_driver` 的最小分层骨架

### 下一步
- 下一小步不建议立刻接 USB
- 更适合先继续理解 `robot_driver` 在系统里的角色
- 或者开始在 `robot_driver` 里加入“把 `Servo18` 转成 42 字节帧”的纯内存打包函数，但先不发送到下位机

---

## 第 5 条记录

### 时间
2026-07-23 19:41:55

### 本次新增 / 修改的文件
- `src/robot_driver/src/main.cpp`
- `LEARNING_LOG.md`

### 本次加了什么
- 在 `robot_driver_node` 里加入了一个纯内存打包函数 `pack_frame(...)`
- 让它把 `Servo18` 映射成下位机要求的 42 字节协议帧
- 新增了一个辅助函数，把 42 字节帧打印成十六进制字符串，方便观察
- 当前还没有发送到 USB，只是在 driver 里完成“组包并打印”

### 这一步主要学到什么
- `robot_driver` 和普通 subscriber 的真正差异，开始体现在“把 ROS2 消息映射成协议字节帧”上
- 42 字节协议帧由固定帧头、长度、seq、18 路小端 `int16`、XOR 校验和固定帧尾组成
- `900` 这个角度值在协议里不是直接放一个数，而是按小端拆成两个字节：`84 03`
- driver 的本质不是“听消息”，而是“把结构化消息翻译成下位机能执行的原始字节数据”

### 当前状态
- `spider_task_node` 仍然持续发布 `/spider/servo_target`
- `robot_driver_node` 现在不仅能收到消息，还能打印组好的 42 字节协议帧
- 例如全 `900` 时，帧里会看到重复的 `84 03`，这说明 18 路角度已经按小端正确展开
- 当前工程已经从“driver 骨架”进入“driver 开始具备协议层行为”的阶段

### 下一步
- 下一小步仍然不建议立刻接 USB
- 更适合先把 `pack_frame(...)` 再逐段讲透，确保你真的理解每个字节从哪里来
- 等你完全理解后，再考虑把这 42 字节真正发到下位机

---

## 第 7 条记录

### 时间
2026-07-25

### 本次新增 / 修改的文件
- `src/leg_calc/include/leg_calc/common_types.hpp`
- `src/leg_calc/include/leg_calc/servo18_mapper.hpp`
- `src/leg_calc/src/main.cpp`
- `src/spider/config/leg_params.yaml`
- `LEARNING_LOG.md`
- `CURRENT_SUMMARY.md`

### 本次加了什么
- 把 `leg_calc` 里的公共类型从“单腿数学类型”继续推进到了“六足静态组织骨架”
- 在 `common_types.hpp` 里补了六条腿统一常量、腿序枚举、左右腿判断和六足足端目标容器 `SpiderFootTargets`
- 把 `LegId` 从 `servo18_mapper.hpp` 挪到更基础的公共类型层，让六足腿序不再只属于“舵机映射那一层”
- 把 `leg_calc_node` 从“只演示单腿/少数腿”改成“组织六条腿默认静态足端目标 -> 逐条腿做 IK -> 汇总成 `SpiderJointTargets` -> 再映射到 18 路 `Servo18` 数组”
- 给 `leg_params.yaml` 增加了当前最小静态组织参数：机体高度、左右腿横向位置、前中后腿纵向位置
- 让 `leg_calc_node` 直接从 `leg_params.yaml` 读取这些最小布局参数，而不是只写死一个 `neutral_foot_target`
- 运行后已经能看到 6 条腿都参与计算，并输出完整 18 路映射结果

### 这一步主要学到什么
- “六足静态组织骨架”本质上不是先做步态，而是先把“六条腿默认怎么摆、数据怎么装、结果怎么统一输出”这层组织好
- 单腿 IK 能跑，只说明数学核心存在；只有把 6 条腿一起组织起来，才开始接近真正的六足机器人上位机结构
- 公共枚举和公共容器应该尽量放在更基础的公共类型层，这样后面数学层、映射层、控制层都能复用
- 配置文件的意义不只是“存参数”，更重要的是把本来会写死在代码里的静态布局抽出来，减少以后反复改源码
- 从日志上看，前腿、中腿、后腿的默认足端布局已经区分出来了，这说明当前代码已经不是“单腿 demo 拼六次”，而是开始有了真正六足静态组织的味道

### 当前状态
- `leg_calc` 现在已经具备：
  - 单腿 IK / FK 核心
  - 六足腿序公共类型
  - 六足默认足端布局组织
  - 六腿关节结果汇总
  - 映射到 18 路 `Servo18` 数组
- 当前 `leg_calc_node` 启动时，6 条腿都会给出各自的默认足端目标、IK 结果、FK 回代结果和最终 18 路舵机数组
- 这说明工程已经从“单腿数学演示 + 映射骨架”推进到“六足静态组织骨架 + 配置驱动的默认布局”阶段
- 但它现在仍然主要是教学型演示节点，还没有真正把 `leg_calc` 的结果发布回 `/spider/servo_target` 主链

### 下一步
- 下一小步最适合把 `leg_calc` 真正接到消息主链，让它能发布 `Servo18`，而不是只打印日志
- 再下一步可以继续补“六足默认站立姿态”和“高层 `/spider/task_cmd_vel` 输入”之间更明确的对应关系
- 后面再逐步往“静态骨架 -> 简单站立控制 -> 更完整多腿组织 -> 步态”推进

---
