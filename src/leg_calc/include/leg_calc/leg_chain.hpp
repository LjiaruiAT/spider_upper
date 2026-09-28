#pragma once

#include <kdl/chain.hpp>

namespace leg_calc {

// 构造单腿的 KDL 链。
//
// 拓扑（从腿坐标系的根往足端长出去）：
//
//   segment "coxa"  : 关节 RotZ(q0) + 连杆 coxa_length   —— 水平转向
//   segment "femur" : 关节 RotY(q1) + 连杆 femur_length  —— 大腿俯仰
//   segment "tibia" : 关节 RotY(q2) + 连杆 tibia_length  —— 膝盖，终点即足端
//
// 于是足端位置是：
//   p = Rz(q0) · ( L_coxa·x̂ + Ry(q1) · ( L_femur·x̂ + Ry(q2) · (L_tibia·x̂) ) )
//
// 注意 q1 / q2 都是绕腿坐标系的 y 轴转，两个角**同号叠加**：
// q1 变大时大腿向下转，q2 变大时膝盖继续往同方向折。
//
// 可达半径范围是 [|coxa − (femur+tibia)|, coxa + femur + tibia]
// （当 coxa 落在 [|femur−tibia|, femur+tibia] 内时下界为 0）。
// 这只是**径向必要条件**：解是否可信最终仍以 FK 回代误差为准。
//
// 三个长度是**相邻铰链轴之间的距离**，直接用机械图纸的值填入，单位米。
// 参数故意不给默认值——腿长是机械属性、属于配置：节点从 leg_params.yaml
// 读出来再传进来，测试则直接构造想要的链。这样"配置"和"代码"各只有一处
// 真相，不会出现两边不一致却都不报错的情况。
KDL::Chain build_leg_chain(double coxa_length_m, double femur_length_m, double tibia_length_m);

}  // namespace leg_calc
