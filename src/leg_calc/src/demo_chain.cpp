#include "leg_calc/demo_chain.hpp"

#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

namespace leg_calc {

KDL::Chain build_demo_chain() {
    // 演示模型，不是真实蜘蛛腿参数：
    //   joint1: RotZ  -> coxa/yaw，改变腿在水平面的方向
    //   joint2: RotY  -> femur/pitch
    //   joint3: RotY  -> tibia/knee
    // 后面的无关节 segment 只提供固定末端几何偏移。
    //
    // 后两段合并后的长度 = |[0.22, 0, -0.12]| = 0.25060 m，
    // 配上第一段的 0.06 m，得到可达球壳半径 [0.19060, 0.31060] m。
    // 改这里的常量时，必须同步重算可达区间，并检查默认站姿是否还在里面。
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(
        "joint1",
        KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
    chain.addSegment(KDL::Segment(
        "joint2",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.06, 0.0, 0.0))));
    chain.addSegment(KDL::Segment(
        "joint3",
        KDL::Joint(KDL::Joint::RotY),
        KDL::Frame(KDL::Vector(0.12, 0.0, -0.02))));
    chain.addSegment(KDL::Segment(
        "foot",
        KDL::Joint(KDL::Joint::None),
        KDL::Frame(KDL::Vector(0.10, 0.0, -0.10))));
    return chain;
}

}  // namespace leg_calc
