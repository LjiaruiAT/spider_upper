// LegKinematics 单元测试
//
// 这一层是整个数学链的几何真相：KDL 链决定 f(q)，IK/FK 都围绕它工作。
// 最容易出问题、也最难靠日志发现的两件事是：
//
//   1. 目标点根本不可达，但求解器不会报"无解"——它会返回"最接近"的位置，
//      配合正常的返回码，看起来像成功了。
//   2. IK 的返回码不是可信度证明，必须用 FK 回代验证。
//
// 本文件测的是 LegKinematics 这个**类的行为**，因此刻意用一条
// 自造参数的链，不绑定任何真实的机械尺寸——真实腿几何的测试在
// test_leg_chain.cpp 里。

#include <gtest/gtest.h>

#include <cmath>

#include "leg_calc/leg_chain.hpp"
#include "leg_calc/leg_kinematics.hpp"

namespace {

using leg_calc::JointVector;
using leg_calc::LegKinematics;

constexpr double kPi = 3.14159265358979323846;

// 测试用链：coxa 0.06 / femur 0.12 / tibia 0.14（米）。
// 可达集合可以解析写出来——末端两段整体挂在距原点 0.06 处，绕 y 轴自由转动，
// 所以可达点满足 |p − (0.06, 0, 0)| ∈ [0.02, 0.26]：一个**圆盘**，不是球壳。
// 全展长（q=0）时足端落在 (0.32, 0, 0)。
LegKinematics make_kinematics() {
    return LegKinematics(leg_calc::build_leg_chain(0.06, 0.12, 0.14));
}

struct SolveOutcome {
    Eigen::Vector3d achieved{Eigen::Vector3d::Zero()};  // FK 回代得到的足端位置
    int code{-1};                                       // KDL 返回码
    double error_norm{0.0};                             // ||achieved - target||
};

// 解一次 IK，并立刻用 FK 回代检查"到底走到了哪里"。
// 只用返回码判断成功与否是不够的——必须看回代误差。
SolveOutcome solve(LegKinematics& kinematics, const Eigen::Vector3d& target) {
    SolveOutcome outcome;
    const JointVector joints = kinematics.inverse_position(target, &outcome.code);
    outcome.achieved = kinematics.forward_position(joints);
    outcome.error_norm = (outcome.achieved - target).norm();
    return outcome;
}

}  // namespace

// ---------------------------------------------------------------------------
// 可达点：IK + FK 回代
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, ReachableTargetIsSolvedAccurately) {
    LegKinematics kinematics = make_kinematics();

    // 0.25 m 落在 [0, 0.32] 内部
    const SolveOutcome outcome = solve(kinematics, Eigen::Vector3d(0.0, 0.0, -0.25));

    EXPECT_GE(outcome.code, 0) << "可达点不应返回错误码";
    EXPECT_LT(outcome.error_norm, 1e-6)
        << "可达点的 FK 回代误差应接近 0，实际 = " << outcome.error_norm * 1000.0 << " mm";
}

// ---------------------------------------------------------------------------
// 不可达点：求解器不会说"无解"，而是落到工作空间边界
//
// 这是最容易被忽视的坑：KDL 会返回一个关节向量，调用方如果只看
// "有没有返回结果"，就会把"够不着"当成"解出来了"。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, UnreachableTargetLandsOnWorkspaceBoundary) {
    LegKinematics kinematics = make_kinematics();

    // 这条链的可达集合不是"以原点为心的球壳"，而是**以髋关节位置为心的圆盘**：
    // 末端两段（femur 0.12 + tibia 0.14）整体挂在距原点 0.06 处，
    // 所以可达点满足 |p − (0.06, 0, 0)| ∈ [|0.12−0.14|, 0.12+0.14]。
    const Eigen::Vector3d femur_joint(0.06, 0.0, 0.0);
    constexpr double kOuterRadius = 0.12 + 0.14;

    // 目标离那个圆心 0.40+ m，超出外径 0.26，物理上伸不到
    const Eigen::Vector3d target(0.0, 0.0, -0.40);
    const SolveOutcome outcome = solve(kinematics, target);

    EXPECT_LT(outcome.code, 0) << "不可达目标应返回错误码";

    // 求解器会把腿伸到最直，也就是停在可达集合的**外边界**上
    EXPECT_NEAR((outcome.achieved - femur_joint).norm(), kOuterRadius, 1e-4)
        << "求解器应该停在可达外边界上，实际停在 " << (outcome.achieved - femur_joint).norm();

    // 误差 = 目标到可达边界的距离，可以直接解析算出来
    const double distance_to_centre = (target - femur_joint).norm();
    EXPECT_NEAR(outcome.error_norm, distance_to_centre - kOuterRadius, 1e-3);
}

// 反向验证：不可达时**不能**报出很小的回代误差。
// 如果哪天有人把判据改成"看返回码"，这条会立刻失败。
TEST(LegKinematicsTest, UnreachableTargetsNeverReportSmallResidual) {
    LegKinematics kinematics = make_kinematics();

    for (const double distance : {0.35, 0.40, 0.60}) {
        const SolveOutcome outcome = solve(kinematics, Eigen::Vector3d(0.0, 0.0, -distance));
        EXPECT_GT(outcome.error_norm, 1e-3)
            << "distance=" << distance << " 超出了工作空间，本次却报出了很小的回代误差";
    }
}

// ---------------------------------------------------------------------------
// 位置偏移必须成对使用
//
// position_offset_ 在 IK 里加上、在 FK 里减掉。只要这一对是对称的，
// 设置非零偏移之后 FK(IK(p)) 仍然应该回到 API 坐标下的 p。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, PositionOffsetIsSymmetricBetweenIkAndFk) {
    LegKinematics kinematics = make_kinematics();
    kinematics.set_position_offset(Eigen::Vector3d(0.0, 0.0, -0.01));

    // 注意：offset 会被加到 IK 的内部目标上，所以这里是 API 坐标系下的点
    const Eigen::Vector3d target(0.0, 0.0, -0.24);
    const SolveOutcome outcome = solve(kinematics, target);

    EXPECT_LT(outcome.error_norm, 1e-6)
        << "带 offset 时 FK(IK(p)) 仍应回到 p（offset 必须在 IK/FK 两侧对称使用）";

    // FK 返回的是 API 坐标，不应包含 offset
    EXPECT_NEAR(outcome.achieved.z(), target.z(), 1e-6);
}

// ---------------------------------------------------------------------------
// 速度映射
//
// v = J(q)·q_dot，inverse_velocity 用最小二乘解回来。
// 在非奇异的构型上，来回一次应该保持一致。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, VelocityMappingRoundTripsAtRegularConfiguration) {
    LegKinematics kinematics = make_kinematics();

    // 一个远离奇异的一般构型
    const JointVector joints = (JointVector() << 0.10, 1.20, 0.60).finished();
    const JointVector joint_vel(0.10, -0.20, 0.30);

    const Eigen::Vector3d foot_vel = kinematics.forward_velocity(joints, joint_vel);
    const JointVector recovered = kinematics.inverse_velocity(joints, foot_vel);

    EXPECT_LT((recovered - joint_vel).norm(), 1e-9)
        << "非奇异构型下 v -> q_dot 的映射应可逆";
}

// ---------------------------------------------------------------------------
// 径向可达预检查
//
// 这里用显式指定的上下界（与链的真实几何无关），测的是 API 的语义。
// 现有节点**不启用**它——真实链带上关节限位后可达集合不再是干净球壳。
// 但接口本身仍要被测，所以这些用例保留。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, ReachLimitsGateTheSolver) {
    LegKinematics kinematics = make_kinematics();
    kinematics.set_reach_limits(0.10, 0.30);

    EXPECT_FALSE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.05))) << "下界以内";
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.20))) << "区间内";
    EXPECT_FALSE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.35))) << "上界以外";
    // 闭区间：正好落在边界上算可达
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.10)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.30)));
}

// 可达半径是**斜边长度**，与方向无关
TEST(LegKinematicsTest, ReachCheckIsDirectionIndependent) {
    LegKinematics kinematics = make_kinematics();
    kinematics.set_reach_limits(0.10, 0.30);

    const double radius = 0.25;
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -radius)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(radius, 0.0, 0.0)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, radius, 0.0)));
}

// 预判必须和 inverse_position 用同一个坐标约定：可达性针对"加上 offset 之后"的目标
TEST(LegKinematicsTest, ReachCheckAccountsForPositionOffset) {
    LegKinematics kinematics = make_kinematics();
    kinematics.set_reach_limits(0.10, 0.30);
    kinematics.set_position_offset(Eigen::Vector3d(0.0, 0.0, -0.01));

    // API 坐标 0.24 + offset 0.01 = 内部目标 0.25，可达
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.24)));
    // 0.05 + 0.01 = 0.06，低于下界
    EXPECT_FALSE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.05)));
}

// 默认不启用预判：任何点都算"在范围内"，行为与加预判之前一致
TEST(LegKinematicsTest, ReachCheckIsDisabledByDefault) {
    LegKinematics kinematics = make_kinematics();

    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.01)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(10.0, 0.0, 0.0)));
}
