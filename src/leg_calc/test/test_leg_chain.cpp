// 真实腿链的几何测试
//
// ⚠ 本文件里的长度 / 角度常量必须与 src/spider/config/leg_params.yaml 保持一致。
//   配置和代码天然是两处真相：改了配置不改这里，测试**不会**失败，
//   但机器人算出来的就是错的。所以这两个地方要一起改。
//
// 这一组用例回答的问题是："我们的 KDL 链和机械图纸是不是同一个构型？"
// 判据是把图纸算出来的关节角喂进我们的 FK，看足端落在哪。

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "leg_calc/leg_chain.hpp"
#include "leg_calc/leg_kinematics.hpp"

namespace {

using leg_calc::JointVector;
using leg_calc::LegKinematics;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// ---- 图纸的连杆长度（米）----
constexpr double kCoxa = 0.0603;
constexpr double kFemur = 0.0895;
constexpr double kTibia = 0.154745;

// ---- 图纸的归位姿态（腿局部坐标系，米）----
// 足端在髋轴外侧 121.8mm、下方 91.4mm
constexpr double kHomeX = 0.1218;
constexpr double kHomeZ = -0.0914;

// ---- 图纸的归位关节角（数学角，度）----
// 换算关系：q0 = θ1, q1 = −θ2, q2 = θ3（θ 是图纸 IK 的约定）。
// 推导见 "工程现状总结.md"。
constexpr double kHomeQ0Deg = 0.0;
constexpr double kHomeQ1Deg = -45.08;
constexpr double kHomeQ2Deg = 135.68;

// ---- 图纸的关节限位（数学角，度）----
constexpr double kCoxaMinDeg = -80.0;
constexpr double kCoxaMaxDeg = 80.0;
constexpr double kFemurMinDeg = -80.0;
constexpr double kFemurMaxDeg = 80.0;
constexpr double kTibiaMinDeg = 50.0;
constexpr double kTibiaMaxDeg = 165.0;

LegKinematics make_kinematics() {
    return LegKinematics(leg_calc::build_leg_chain(kCoxa, kFemur, kTibia));
}

}  // namespace

// ---------------------------------------------------------------------------
// 链的构造
// ---------------------------------------------------------------------------
TEST(LegChainTest, ZeroJointsExtendStraightAlongLocalX) {
    // q = 0 时三个杆应当完全伸直、指向局部 +x。
    // 这一条同时验证了三个杆长都被真正用进了链里。
    LegKinematics kinematics = make_kinematics();
    const Eigen::Vector3d foot = kinematics.forward_position(JointVector::Zero());

    EXPECT_NEAR(foot.x(), kCoxa + kFemur + kTibia, 1e-12) << "全展长必须等于三个杆长之和";
    EXPECT_NEAR(foot.y(), 0.0, 1e-12);
    EXPECT_NEAR(foot.z(), 0.0, 1e-12);
}

TEST(LegChainTest, RejectsNonPositiveLinkLengths) {
    // 杆长为 0 或负数说明配置没填对，必须在构造时就拦下来，
    // 否则会得到一条退化的链，IK 仍会"成功"地返回一些没意义的角度。
    EXPECT_THROW(leg_calc::build_leg_chain(0.0, kFemur, kTibia), std::runtime_error);
    EXPECT_THROW(leg_calc::build_leg_chain(kCoxa, -0.1, kTibia), std::runtime_error);
    EXPECT_THROW(leg_calc::build_leg_chain(kCoxa, kFemur, 0.0), std::runtime_error);
}

// ---------------------------------------------------------------------------
// ★ 核心验收：我们的链和图纸是同一个构型
//
// 把图纸算出来的关节角直接喂进我们的 FK，足端应当正好落在图纸的归位位置。
// 这是"符号约定没搞反"的直接证据——如果 q1 或 q2 的符号反了，
// 足端会跑到别的地方去，误差是厘米级的。
// ---------------------------------------------------------------------------
TEST(LegChainTest, DesignJointAnglesReproduceTheDrawingHomePose) {
    LegKinematics kinematics = make_kinematics();
    const JointVector joints =
        (JointVector() << kHomeQ0Deg * kDeg, kHomeQ1Deg * kDeg, kHomeQ2Deg * kDeg).finished();

    const Eigen::Vector3d foot = kinematics.forward_position(joints);

    EXPECT_NEAR(foot.x(), kHomeX, 1e-4) << "足端的前伸量与图纸不符";
    EXPECT_NEAR(foot.y(), 0.0, 1e-12) << "归位姿态在腿坐标系的 x-z 平面内，y 应为 0";
    EXPECT_NEAR(foot.z(), kHomeZ, 1e-4) << "足端的下探深度与图纸不符";
}

// 图纸的归位姿态本身必须落在关节限位之内，否则机器人一上电就会报警。
// 注意 femur 的区间是对称的（±80°），符号反了看不出来；
// 但 tibia 的 [50°, 165°] 不对称，符号反了立刻超限——所以这条主要防 tibia。
TEST(LegChainTest, DesignHomePoseIsInsideJointLimits) {
    EXPECT_GE(kHomeQ0Deg, kCoxaMinDeg);
    EXPECT_LE(kHomeQ0Deg, kCoxaMaxDeg);
    EXPECT_GE(kHomeQ1Deg, kFemurMinDeg);
    EXPECT_LE(kHomeQ1Deg, kFemurMaxDeg);
    EXPECT_GE(kHomeQ2Deg, kTibiaMinDeg);
    EXPECT_LE(kHomeQ2Deg, kTibiaMaxDeg);
}

// ---------------------------------------------------------------------------
// 归位姿态能被 IK 解出来，且解落在限位内
//
// 这是"机器人能站住"的最小必要条件。如果这条失败，说明初始站位不可达，
// 或者求解器收敛到了一个机械上转不到的分支。
// ---------------------------------------------------------------------------
TEST(LegChainTest, HomePoseIsSolvedAccurately) {
    LegKinematics kinematics = make_kinematics();
    const Eigen::Vector3d home(kHomeX, 0.0, kHomeZ);

    int code = -1;
    const JointVector joints = kinematics.inverse_position(home, &code);
    const Eigen::Vector3d achieved = kinematics.forward_position(joints);

    EXPECT_GE(code, 0);
    EXPECT_LT((achieved - home).norm(), 1e-6) << "归位姿态的 FK 回代误差应接近 0";
}

TEST(LegChainTest, HomePoseSolutionStaysInsideJointLimits) {
    LegKinematics kinematics = make_kinematics();
    const Eigen::Vector3d home(kHomeX, 0.0, kHomeZ);

    int code = -1;
    const JointVector joints = kinematics.inverse_position(home, &code);

    EXPECT_GE(joints(0) * kRadToDeg, kCoxaMinDeg - 1e-6);
    EXPECT_LE(joints(0) * kRadToDeg, kCoxaMaxDeg + 1e-6);
    EXPECT_GE(joints(1) * kRadToDeg, kFemurMinDeg - 1e-6) << "IK 收敛到了机械转不到的分支";
    EXPECT_LE(joints(1) * kRadToDeg, kFemurMaxDeg + 1e-6) << "IK 收敛到了机械转不到的分支";
    EXPECT_GE(joints(2) * kRadToDeg, kTibiaMinDeg - 1e-6) << "IK 收敛到了机械转不到的分支";
    EXPECT_LE(joints(2) * kRadToDeg, kTibiaMaxDeg + 1e-6) << "IK 收敛到了机械转不到的分支";
}

// ---------------------------------------------------------------------------
// 工作空间：比 demo 链大得多，但也不是没有边界
// ---------------------------------------------------------------------------
TEST(LegChainTest, TargetBeyondFullExtensionIsNotReachable) {
    LegKinematics kinematics = make_kinematics();

    // 全展长 0.304545 m，取 0.50 m 一定伸不到
    const Eigen::Vector3d target(0.0, 0.0, -0.50);
    int code = -1;
    const JointVector joints = kinematics.inverse_position(target, &code);
    const Eigen::Vector3d achieved = kinematics.forward_position(joints);

    EXPECT_GT((achieved - target).norm(), 1e-3) << "超出全展长却报出了很小的回代误差";
}
