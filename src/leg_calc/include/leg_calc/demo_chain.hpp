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

}  // namespace leg_calc
