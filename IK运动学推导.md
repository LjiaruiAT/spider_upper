# Spider Upper 单腿 IK 运动学推导

> 这份文档专门解释 `src/leg_calc` 当前代码里的运动学。目标不是只记住“调用哪个函数”，而是从坐标系、几何模型和公式推导出：为什么输入一个足端点，就能得到三个关节角。
>
> 重要前提：当前代码使用的是 `main.cpp` 里的 **demo KDL chain**，不是最终真实蜘蛛腿的几何模型。因此本文先解释“代码现在怎样算”，再说明“真实机器人还需要补什么”。

---

## 1. IK 到底要解决什么问题？

蜘蛛的一条腿有三个关节：

```text
q = [q0, q1, q2]^T
  = [coxa, femur, tibia]^T
```

代码中的关节角统一使用 **弧度 rad**。足端位置使用 **米 m**：

```text
p = [x, y, z]^T
```

正运动学（Forward Kinematics，FK）回答：

> 已知关节角 q，脚尖在哪里？

```text
q  ->  FK  ->  p
```

逆运动学（Inverse Kinematics，IK）回答：

> 已知脚尖目标位置 p，三个关节应该转到多少？

```text
p  ->  IK  ->  q
```

两者的关系可以写成：

```text
p = f(q)
q = f^(-1)(p)       // 概念上的逆函数，实际通常通过数值算法求
```

注意：`f^(-1)` 不一定是一个简单、唯一的公式。一个足端位置可能有多个姿态，例如膝盖向上和膝盖向下；也可能没有解，例如目标点超出腿的最大伸展范围。当前代码使用 KDL 的数值迭代求解器，而不是手写解析解。

---

## 2. 先分清三个坐标系/层次

当前程序至少涉及两个重要坐标系：

### 2.1 身体坐标系 `body`

这是六足整体使用的坐标系，代码中默认 frame 名称是：

```text
spider_base
```

`main.cpp` 会在这里生成六条腿的目标足端位置，例如：

```text
LF: [ 0.18,  0.12, -0.12 ]
RF: [ 0.18, -0.12, -0.12 ]
```

当前约定可以理解为：

```text
x：身体前后方向，前方为正
 y：身体左右方向，左侧为正、右侧为负
 z：竖直方向，身体下方为负
```

### 2.2 单腿局部坐标系 `leg`

每条腿都有自己的安装原点。KDL 的 chain 从单腿局部坐标系开始，因此 IK 的输入不是身体坐标系目标，而是腿坐标系目标。

身体坐标系中的点记为：

```text
p_body
```

腿坐标系中的同一个物理点记为：

```text
p_leg
```

### 2.3 关节空间 `joint`

关节空间向量是：

```text
q = [coxa, femur, tibia]^T
```

IK 做的是从笛卡尔空间回到关节空间：

```text
足端笛卡尔位置 p_leg -> 关节空间 q
```

---

## 3. 身体坐标系和腿坐标系怎么转换？

`common_types.hpp` 中的：

```cpp
Eigen::Isometry3d body_T_leg;
```

表示一个齐次变换。记作：

```text
body_T_leg
```

它的含义是：把腿坐标系中的点变换到身体坐标系：

```text
p_body = body_T_leg * p_leg
```

因此反过来：

```text
p_leg = body_T_leg^(-1) * p_body
```

代码对应：

```cpp
const auto leg_target =
    leg_calc::body_point_to_leg_point(body_target, mount);
```

这个顺序非常重要：

```text
身体层产生目标
    ↓
身体坐标系足端点 p_body
    ↓ body_T_leg^(-1)
单腿坐标系足端点 p_leg
    ↓
IK
    ↓
三个关节角 q
```

### 3.1 为什么中性姿态转换后 x/y 变成 0？

以左前腿 LF 为例：

```text
身体坐标系目标：
 p_body = [0.18, 0.12, -0.12]

左前腿安装原点：
 origin = [0.18, 0.12, 0]
```

由于当前没有 yaw 旋转：

```text
body_T_leg = 平移 [0.18, 0.12, 0]
```

所以：

```text
p_leg = p_body - origin
      = [0.18, 0.12, -0.12] - [0.18, 0.12, 0]
      = [0, 0, -0.12]
```

这表示：中性姿态下，脚尖在自己的腿坐标系里正下方 12 cm。

---

## 4. 当前 demo KDL 链是什么？

当前 `main.cpp` 的 `build_demo_chain()` 创建了四个 segment，其中前三个有旋转关节：

```cpp
joint1: RotZ
joint2: RotY
joint3: RotY
foot:   Joint::None
```

可以抽象成：

```text
腿基座
  └─ q0: 绕 Z 轴旋转（RotZ）
      └─ 固定平移 [0.06, 0, 0]
          └─ q1: 绕 Y 轴旋转（RotY）
              └─ 固定平移 [0.12, 0, -0.02]
                  └─ q2: 绕 Y 轴旋转（RotY）
                      └─ 固定平移 [0.10, 0, -0.10]
```

这里要特别小心：KDL 中 `Joint` 和随后 `Segment` 的 `Frame` 共同决定变换链。不能只看“0.06、0.12、0.10”就简单断言它们分别是某一根真实连杆的长度；它们当前只是 demo 几何参数。

代码中的链可以用变换乘积表示：

```text
T_leg_foot(q)
  = T_joint1(q0)
  · T_fixed1
  · T_joint2(q1)
  · T_fixed2
  · T_joint3(q2)
  · T_fixed3
```

正运动学就是从根坐标系一路乘到足端：

```text
T_leg_foot(q) = Π_i T_i(q_i)
```

足端位置是这个齐次变换的平移部分：

```text
p_leg = translation(T_leg_foot(q))
```

KDL 的 `ChainFkSolverPos_recursive` 负责实际执行这个变换乘积。

---

## 5. 先理解第一关节 q0：水平转向

第一关节是：

```text
q0: RotZ
```

它绕 z 轴旋转，所以它主要改变腿在水平面上的朝向。

二维平面绕 z 轴旋转的矩阵是：

```text
Rz(q0) = [ cos(q0)  -sin(q0)  0 ]
         [ sin(q0)   cos(q0)  0 ]
         [    0         0     1 ]
```

如果后面的二连杆在某个局部平面内伸出，q0 会把这个平面整体绕 z 轴转动。

因此，从三个坐标看：

- `q0` 主要控制足端的水平角度，也就是 `atan2(y, x)` 方向。
- `q1`、`q2` 主要决定腿在这个方向上的伸展和高度。

在一个理想、对称的三自由度蜘蛛腿里，第一步通常可以先求：

```text
q0 ≈ atan2(y, x)
```

但是当前代码没有手写这个公式，而是把整个三关节链交给 KDL 的数值 IK 统一求解。

---

## 6. 理想三连杆几何中的 q1/q2 推导

为了理解原理，先看常见的简化蜘蛛腿模型。设：

- `L1`：coxa 的水平长度
- `L2`：femur 长度
- `L3`：tibia 长度
- `q0`：coxa 绕 z 的角度
- `q1`：femur 俯仰角
- `q2`：tibia 相对 femur 的俯仰角

这是一种教学模型。真实机器人可能有角度零点、方向、偏置和镜像差异。

### 6.1 先把三维问题降成二维

目标足端为：

```text
p = [x, y, z]^T
```

第一关节先决定水平朝向：

```text
q0 = atan2(y, x)
```

然后计算足端到第一关节轴的水平距离：

```text
r = sqrt(x^2 + y^2)
```

coxa 占掉 `L1` 后，femur/tibia 二连杆还需要覆盖的水平距离是：

```text
rho = r - L1
```

于是 q1/q2 只需要在 `rho-z` 平面内解决：

```text
目标 = [rho, z]
```

### 6.2 二连杆的几何方程

如果 q2 是 tibia 相对于 femur 的夹角，则：

```text
rho = L2*cos(q1) + L3*cos(q1 + q2)
z   = L2*sin(q1) + L3*sin(q1 + q2)
```

这里的符号取决于坐标轴和关节正方向。你在真实机器人上看到的正负号可能不同，但“两个杆件向量相加”的结构不变。

定义目标距离：

```text
D^2 = rho^2 + z^2
```

由余弦定理：

```text
D^2 = L2^2 + L3^2 + 2*L2*L3*cos(q2)
```

所以：

```text
cos(q2) = (D^2 - L2^2 - L3^2) / (2*L2*L3)
```

为了有几何解，必须满足：

```text
-1 <= cos(q2) <= 1
```

等价地，目标距离必须满足：

```text
|L2 - L3| <= D <= L2 + L3
```

这就是最基本的可达性判断。

### 6.3 q2 的两个分支

由反余弦可以得到两个常见分支：

```text
q2 = +acos(cos_q2)   // 一种膝盖方向
q2 = -acos(cos_q2)   // 另一种膝盖方向
```

也就是说，同一个足端点经常有两套甚至更多关节角解。工程上通常根据：

- 机械关节限位
- 膝盖应该朝上还是朝下
- 上一时刻的关节角
- 是否发生跳变

选择合适分支。

### 6.4 q1 的推导

令：

```text
phi = atan2(z, rho)
```

这是“目标方向角”。再令：

```text
beta = atan2(L3*sin(q2), L2 + L3*cos(q2))
```

则：

```text
q1 = phi - beta
```

因此理想解析 IK 的主要步骤是：

```text
q0 = atan2(y, x)
r  = sqrt(x^2 + y^2)
rho = r - L1
D2 = rho^2 + z^2
cos_q2 = (D2 - L2^2 - L3^2) / (2*L2*L3)
q2 = ±acos(cos_q2)
phi = atan2(z, rho)
beta = atan2(L3*sin(q2), L2 + L3*cos(q2))
q1 = phi - beta
```

当前代码没有直接实现这些公式，因为它使用 KDL 的通用数值链；但这组公式能帮助你理解数值 IK 最终想完成的事情。

---

## 7. 当前代码为什么使用数值 IK？

在 `LegKinematics` 构造函数里：

```cpp
ik_solver_(
    chain_,
    Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
    1e-6,
    150,
    1e-10)
```

这是 KDL 的 Levenberg-Marquardt（LMA）位置 IK 求解器。

### 7.1 它在优化什么？

KDL 链给出正运动学：

```text
p_current = f(q)
```

目标是：

```text
p_target
```

位置误差为：

```text
e(q) = p_target - f(q)
```

数值 IK 不直接一次算出 q，而是从一个初始关节角开始，反复更新：

```text
q_(k+1) = q_k + Δq_k
```

直到：

```text
||e(q)|| 足够小
```

或者达到最大迭代次数。

### 7.2 为什么权重有六个？

KDL 的笛卡尔位姿一般包括 6 个任务维度：

```text
[x, y, z, rotation_x, rotation_y, rotation_z]
```

当前权重是：

```text
[1, 1, 1, 0, 0, 0]
```

含义：

```text
关心足端位置 x/y/z
不关心足端姿态 rotation_x/y/z
```

因此当前是“位置 IK”，不是完整的“位姿 IK”。`target_frame.M = Identity()` 只是填入一个姿态值；由于旋转权重为 0，这个姿态不会参与优化目标。

### 7.3 LMA 和 Jacobian 的关系

在当前关节角附近，可以用一阶近似：

```text
f(q + Δq) ≈ f(q) + J(q) Δq
```

其中 `J(q)` 是位置 Jacobian：

```text
J(q) = ∂p / ∂q
```

于是希望：

```text
J(q) Δq ≈ e
```

LMA 会在最小二乘和稳定阻尼之间做折中，计算一个合适的 `Δq`。它不断重复这个过程，直到收敛或失败。

---

## 8. `inverse_position()` 逐步对应什么公式？

代码核心：

```cpp
JointVector LegKinematics::inverse_position(
    const Eigen::Vector3d& foot_pos,
    int* result) {
```

### 第一步：处理 offset

```cpp
const Eigen::Vector3d target = foot_pos + position_offset_;
```

当前默认：

```text
position_offset_ = [0, 0, 0]
```

所以现在通常：

```text
target = foot_pos
```

但从接口设计看，KDL 内部目标点是：

```text
target_KDL = target_API + position_offset
```

FK 中又做相反操作：

```text
position_API = position_KDL - position_offset
```

这样 IK 和 FK 的坐标约定保持对称。

### 第二步：构造 KDL 目标

```cpp
target_frame.p = KDL::Vector(target.x(), target.y(), target.z());
target_frame.M = KDL::Rotation::Identity();
```

位置写入 `[x,y,z]`，姿态填单位旋转。

### 第三步：选择初始值

```cpp
KDL::JntArray solution = last_joint_solution_;
```

以及：

```cpp
ik_solver_.CartToJnt(last_joint_solution_, target_frame, solution);
```

上一周期的结果作为这一次的初始猜测：

```text
q_initial = q_last
```

这叫 **warm start（暖启动）**。

它有两个重要效果：

1. 连续运动时通常只需要小幅修正，收敛更快。
2. 在有多个 IK 分支时，更倾向于留在上一周期附近，减少关节跳变。

### 第四步：迭代求解

KDL 内部大致重复：

```text
p_current = FK(q)
e = p_target - p_current
J = ∂p/∂q
求 Δq，使 JΔq ≈ e
q = q + Δq
```

### 第五步：处理结果码

```cpp
if (*result >= 0) {
    last_joint_solution_ = solution;
}
```

成功时保存新结果，用于下一次暖启动。

失败时不更新缓存，这样缓存仍保留上一次成功的解。

但当前调用方 `main.cpp` 仍然把返回的 `joint_solution` 写入输出，只在日志里打印 `ik_result`，没有进一步：

- 拒绝失败结果
- 保留上一帧输出
- 计算并检查足端误差
- 检查关节限位

因此实际工程中还需要增强失败处理。

---

## 9. FK 如何验证 IK？

`solve_joint_targets()` 中先做：

```cpp
const auto joint_solution =
    kinematics_->inverse_position(leg_target, &ik_result);
```

得到：

```text
q_ik
```

然后做：

```cpp
const auto reconstructed_position =
    kinematics_->forward_position(joint_solution);
```

得到：

```text
p_reconstructed = FK(q_ik)
```

最后再转换回身体坐标系：

```cpp
const auto reconstructed_body_position =
    leg_calc::leg_point_to_body_point(reconstructed_position, mount);
```

理想情况下应该满足：

```text
p_reconstructed_leg ≈ p_target_leg
```

或在身体坐标系中：

```text
p_reconstructed_body ≈ p_target_body
```

真正应该计算的误差是：

```text
error = p_reconstructed - p_target
error_norm = sqrt(error.x^2 + error.y^2 + error.z^2)
```

当前代码只是打印两者，没有主动计算 `error_norm`。

### 当前运行中的重要现象

当前 demo 链运行时，日志可能出现：

```text
ik=-101
joints=[0.0000, -1.5708, 2.6422]
fk_body=[..., ..., -0.1906]
```

KDL 中 `-101` 对应：

```text
E_INCREMENT_JOINTS_TOO_SMALL
```

含义是：关节增量已经很小，但求解器没有按成功条件结束。它不应简单等同于“目标已经准确到达”。日志里目标 z 是 `-0.1200`，而 FK 回代 z 约为 `-0.1906`，说明当前 demo 目标和 demo 链之间存在明显误差。

这恰好说明：

> IK 求解器返回一个关节向量，不代表这个向量一定让真实脚尖到达目标；必须同时看返回码和 FK 回代误差。

---

## 10. Jacobian 是什么？

Jacobian 是“关节速度如何影响足端速度”的局部线性映射：

```text
v = J(q) q_dot
```

其中：

```text
q_dot = [q0_dot, q1_dot, q2_dot]^T   // rad/s
v     = [vx, vy, vz]^T              // m/s
```

位置 Jacobian：

```text
J(q) = [ ∂x/∂q0  ∂x/∂q1  ∂x/∂q2 ]
       [ ∂y/∂q0  ∂y/∂q1  ∂y/∂q2 ]
       [ ∂z/∂q0  ∂z/∂q1  ∂z/∂q2 ]
```

它是一个 3×3 矩阵。每一列回答：

> 只让第 j 个关节以单位角速度运动，足端线速度会怎样？

### 10.1 速度正运动学

代码：

```cpp
return jacobian * joint_vel;
```

对应：

```text
v = J(q) q_dot
```

输入当前角度和关节角速度，输出足端线速度。

### 10.2 速度逆运动学

代码：

```cpp
return jacobian.completeOrthogonalDecomposition().solve(foot_vel);
```

对应求解：

```text
J(q) q_dot ≈ v_target
```

如果 `J` 可逆，理论上：

```text
q_dot = J^(-1) v_target
```

但直接求逆可能在奇异位置附近非常不稳定，所以当前代码使用 Eigen 的完全正交分解，按最小二乘意义寻找解。

当前实现还没有加入：

- 关节速度限制
- 阻尼最小二乘
- 奇异性阈值报警
- 任务优先级
- 姿态速度

### 10.3 为什么只取 Jacobian 前三行？

KDL 完整 Jacobian 有 6 行：

```text
前 3 行：线速度 vx, vy, vz
后 3 行：角速度 wx, wy, wz
```

`extract_position_jacobian()` 只取前 3 行，因此当前只处理足端位置速度，不处理脚的姿态速度。

---

## 11. 步态和轨迹怎样把目标送入 IK？

IK 不会自己决定脚往哪里走。它只负责：

```text
给定足端目标 -> 求关节角
```

目标由上游生成：

```text
cmd_vel
  -> GaitPhaseManager
  -> FootTrajectory
  -> body_foot_targets
  -> body_point_to_leg_point
  -> inverse_position
```

### 11.1 步态相位

三足步态中：

```text
global_phase ∈ [0,1)
```

前半周期：

```text
Tripod A = Stance
Tripod B = Swing
```

后半周期：

```text
Tripod A = Swing
Tripod B = Stance
```

每条腿的：

```text
phase_fraction ∈ [0,1)
```

会被重新拉伸到当前支撑相或摆动相内部。

### 11.2 支撑相

当前代码使用：

```cpp
centered = 1 - 2*fraction;
```

所以：

```text
fraction = 0   -> centered = +1
fraction = 0.5 -> centered =  0
fraction = 1   -> centered = -1
```

支撑相中，脚相对身体向后移动，用来模拟身体向前走：

```text
target = nominal - planar_motion
```

### 11.3 摆动相

摆动相中，脚抬离地面并向前摆：

```cpp
progress = 2*fraction - 1
lift = sin(pi*fraction)
```

抬脚高度：

```text
z_target = z_nominal + step_height * sin(pi*fraction)
```

因为：

```text
sin(0)=0
sin(pi/2)=1
sin(pi)=0
```

所以脚在摆动相起点和终点落在地面高度，中间抬到最高。

---

## 12. IK 输出之后还不是舵机协议角度

IK 输出的是每条腿的关节弧度：

```text
[q_coxa, q_femur, q_tibia]
```

接下来经过 `Servo18Mapper`：

```text
6 条腿 × 3 个关节
    -> 18 个 Servo18 通道
```

单位转换：

```text
angle_ddeg = round(rad * 180/pi * 10)
```

例如：

```text
90° = pi/2 rad
angle_ddeg = 90 * 10 = 900
```

所以要区分：

```text
IK：足端位置 -> 数学关节角（rad）
Mapper：数学关节角（rad） -> 舵机通道/协议单位（0.1°）
```

真实机器人还需要在 mapper 中考虑：

```text
舵机零点 offset
舵机方向 direction = ±1
关节限幅
不同型号舵机范围
```

当前这些还没有完整实现。

---

## 13. 当前实现中最关键的限制

### 13.1 demo chain 不是真实腿

`build_demo_chain()` 使用固定常量：

```text
0.06
0.12
-0.02
0.10
-0.10
```

而 `leg_params.yaml` 中虽然有：

```yaml
coxa_length_mm
femur_length_mm
tibia_length_mm
```

但当前这些腿长还没有真正用于创建 KDL chain。

### 13.2 六条腿共用同一条链

当前六条腿只共享一个：

```cpp
std::shared_ptr<leg_calc::LegKinematics> kinematics_;
```

没有为左右侧建立不同的镜像链，也没有根据每条腿的安装方向创建不同链。

### 13.3 没有关节限位

数学上求出的角度可能不适合机械结构。真实 IK 应该检查：

```text
q_min <= q <= q_max
```

### 13.4 没有正式可达性检查

目标点超出工作空间时，应该提前判断或根据 IK 失败拒绝输出。

### 13.5 失败结果仍可能被继续使用

当前 main 里虽然记录：

```cpp
int ik_result = -1;
```

但没有用它阻止：

```cpp
spider_targets.legs[index].joints = joint_solution;
```

后续完善时，应至少做到：

```text
IK 失败 -> 保留上一帧安全角度 / 停止该腿输出 / 报警
```

### 13.6 `velocity_solver_` 当前没有被使用

类里创建了：

```cpp
KDL::ChainIkSolverVel_pinv velocity_solver_;
```

但 `inverse_velocity()` 当前是自己算 Jacobian，再用 Eigen 求解。因此这个成员目前属于预留/未使用状态。

---

## 14. 学习和调试 IK 的推荐顺序

建议按下面的顺序学习，不要一上来就看全部 ROS2 代码：

### 第一步：只看坐标变换

手算：

```text
p_body = [0.18,0.12,-0.12]
origin = [0.18,0.12,0]
p_leg = [0,0,-0.12]
```

先确保明白“为什么身体坐标变腿坐标”。

### 第二步：只看 KDL chain

画出：

```text
RotZ -> fixed frame -> RotY -> fixed frame -> RotY -> foot frame
```

确认每个关节绕哪个轴。

### 第三步：理解 FK

先给一个 q：

```text
q = [0, -pi/2, 2.64]
```

让程序算：

```text
p = FK(q)
```

### 第四步：理解 IK

给定一个 p：

```text
p = [0,0,-0.12]
```

让程序从初值 q 开始迭代求解。

### 第五步：做 FK 回代

检查：

```text
FK(IK(p)) ≈ p
```

不能只看关节角，更不能只看 IK 返回了一个整数结果。

### 第六步：最后看步态

明白：

```text
步态/轨迹负责产生 p(t)
IK 负责把 p(t) 转成 q(t)
```

---

## 15. 一句话总结

当前 `leg_calc` 的核心可以压缩成下面这条链：

```text
身体坐标系足端目标 p_body
    ↓ body_T_leg^(-1)
腿坐标系目标 p_leg
    ↓ KDL LMA 数值 IK
关节角 q=[coxa,femur,tibia]
    ↓ KDL FK 回代
检查 FK(q) 是否接近 p_leg
    ↓ Servo18Mapper
18 路 0.1° 舵机目标
```

真正理解 IK，不是只知道：

```cpp
inverse_position(...)
```

而是要能回答：

1. 目标点在哪个坐标系？
2. KDL chain 的根坐标系是什么？
3. 三个关节分别绕哪个轴？
4. FK 的 `f(q)` 是由哪些变换相乘得到的？
5. IK 优化的误差是什么？
6. 初始关节角从哪里来？
7. 多解和不可达时怎么办？
8. 如何用 FK 回代验证？
9. 数学弧度如何变成实际舵机角度？

只要这九个问题能说清楚，就真正掌握了当前这套 IK 的计算逻辑。
