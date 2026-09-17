#pragma once

#include <kdl/chain.hpp>

namespace leg_calc {

// 构造当前"演示用"的单腿 KDL 链。
//
// 重要：这**不是**真实蜘蛛腿的几何。
//   joint1: RotZ  -> coxa / 水平转向（腿朝哪个方位伸）
//   joint2: RotY  -> femur / 大腿俯仰
//   joint3: RotY  -> tibia / 膝盖
//   后面的无关节 segment 只提供固定的末端几何偏移。
//
// 链里的杆长常量目前仍是写死的演示值，`leg_params.yaml` 里的
// coxa/femur/tibia_length_mm 还没有接入。因此不要把这条链解出的角度
// 直接当作真实舵机的安装角；后续应让链结构与机械图纸一致。
//
// 这条链的可达工作空间是**半径 190.6mm ~ 310.6mm 的球壳**，
// 所以默认站姿的足端距离 |p| 必须落在这个区间内，否则 IK 无解，
// 求解器只会返回"最接近"的位置（表现为持续的 FK 回代误差）。
//
// 之所以把它从节点里抽出来，是为了让**节点和单元测试使用同一条链**，
// 否则测试验证的就不是实际运行时的那条几何。
KDL::Chain build_demo_chain();

// demo 链的径向可达区间（米），对应 LegKinematics::set_reach_limits()。
//
// 推导（详见 工程现状总结.md 5.3 节）：
//   joint3 的平移 [0.12, 0, -0.02] 和 foot 的平移 [0.10, 0, -0.10] 之间没有关节，
//   是刚性相连的，合并后长度 = |[0.22, 0, -0.12]| = 0.250599 m；
//   第一段长度 0.06 m。两个向量反向 => 最小可达 0.250599 − 0.06 = 0.190599 m；
//   同向 => 最大可达 0.250599 + 0.06 = 0.310599 m。
//   q0 绕 z 轴旋转把这段区间扫成一个**球壳**，所以可达性可以用半径判断。
//
// 改链里的任何一个常量，这两个值都必须重算。
// test_leg_kinematics.cpp 会用密集采样 FK 验证它们与链保持一致，忘了改会被测试拦下。
inline constexpr double kDemoChainMinReachM = 0.190599282;
inline constexpr double kDemoChainMaxReachM = 0.310599282;

}  // namespace leg_calc
