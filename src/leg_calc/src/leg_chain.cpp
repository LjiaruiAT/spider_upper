#include "leg_calc/leg_chain.hpp"

#include <cmath>
#include <stdexcept>

#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

namespace leg_calc {

KDL::Chain build_leg_chain(double coxa_length_m, double femur_length_m, double tibia_length_m) {
    if (!(coxa_length_m > 0.0) || !(femur_length_m > 0.0) || !(tibia_length_m > 0.0)) {
        throw std::runtime_error("leg link lengths must be positive");
    }

    // KDL 的 Segment 约定：f_tip 是"从这个关节到**下一个**关节"的固定变换，
    // 也就是这个关节所带动的那根连杆。所以杆长必须写在
    // "以该连杆近端关节开头的那个 segment" 里：
    //
    //   segment "coxa"  的 joint 是 q0，f_tip 走完 coxa 这根杆
    //   segment "femur" 的 joint 是 q1，f_tip 走完 femur 这根杆
    //   segment "tibia" 的 joint 是 q2，f_tip 走完 tibia 这根杆（终点即足端）
    //
    // 于是足端位置是：
    //   p = Rz(q0) · ( L_coxa·x̂
    //                  + Ry(q1) · ( L_femur·x̂ + Ry(q2) · (L_tibia·x̂) ) )
    //
    // ⚠ 这一步很容易写错：如果把三个长度**整体往后挪一格**
    //   （coxa 段留空、femur 段填 coxa 长度、tibia 段填 femur 长度、再加一个
    //     无关节的 foot 段装 tibia 长度），那么
    //     · 总展长不变
    //     · q = 0 时的足端位置也不变
    //   但两个关节的**位置**全错了，FK/IK 的结果会整体偏移几厘米。
    //   test_leg_chain.cpp 里"图纸关节角应还原图纸归位姿态"那条就是专门拦它的。
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(
        "coxa",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(coxa_length_m, 0.0, 0.0))));
    chain.addSegment(KDL::Segment(
        "femur",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(femur_length_m, 0.0, 0.0))));
    chain.addSegment(KDL::Segment(
        "tibia",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(tibia_length_m, 0.0, 0.0))));
    return chain;
}

}  // namespace leg_calc
