// LegKinematics 单元测试
//
// 这一层是整个数学链的几何真相：KDL 链决定 f(q)，IK/FK 都围绕它工作。
// 最容易出问题、也最难靠日志发现的两件事是：
//
//   1. 目标点根本不可达，但求解器不会报"无解"——它会返回"最接近"的位置，
//      配合正常的返回码，看起来像成功了。
//   2. IK 的返回码不是可信度证明，必须用 FK 回代验证。
//
// 本文件把这两条钉成断言，并把 demo 链的可达工作空间（球壳）写成测试。
//
// 注意：这里测的是 demo 链，不是真实机械腿。等真实链结构确定后，
// 需要同步更新可达区间常量，并确认 IK 仍在合理构型下收敛。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "leg_calc/demo_chain.hpp"
#include "leg_calc/leg_kinematics.hpp"

namespace {

using leg_calc::JointVector;
using leg_calc::LegKinematics;

constexpr double kPi = 3.14159265358979323846;

// demo 链的几何：把后两段合并成一个定长向量
//   joint3 的平移 [0.12, 0, -0.02] + foot 的平移 [0.10, 0, -0.10]
//   = [0.22, 0, -0.12]，长度 0.25060 m
// 第一段长度 0.06 m，所以两个向量之和的长度范围是：
//   最小 |0.25060 - 0.06| = 0.19060 m   （两向量反向）
//   最大 0.25060 + 0.06   = 0.31060 m   （两向量同向）
// 再考虑 q0 绕 z 轴旋转，可达集合就是半径落在这个区间的**球壳**。
const double kFirstLinkM = 0.06;
const double kSecondLinkM = std::sqrt(0.22 * 0.22 + 0.12 * 0.12);
const double kMinReachM = kSecondLinkM - kFirstLinkM;
const double kMaxReachM = kSecondLinkM + kFirstLinkM;

// 节点运行时用的是 demo_chain.hpp 里那两个常量。它们必须和链的真实几何一致，
// 所以这里把它们和上面推导出来的值对照一次；下面还有一条密集采样测试，
// 用来拦住"改了链却忘了改常量"这种情况。

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
// 正运动学的手算对照
//
// 用文档里反复出现的那组角验证 FK：
//   q = [0, -pi/2, 2.6422]
// 手算过程：
//   Ry(-pi/2)·[0.06,0,0]              = [0, 0, 0.06]
//   Ry(2.6422)·[0.22,0,-0.12]         ≈ [-0.2506, 0, 0.0000]
//   Ry(-pi/2) 作用于它                 ≈ [0.0000, 0, -0.2506]
//   合计 ≈ [0, 0, -0.1906]
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, ForwardKinematicsMatchesHandCalculation) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    const JointVector joints = (JointVector() << 0.0, -kPi / 2.0, 2.6422).finished();
    const Eigen::Vector3d foot = kinematics.forward_position(joints);

    EXPECT_NEAR(foot.x(), 0.0, 1e-4);
    EXPECT_NEAR(foot.y(), 0.0, 1e-12);
    EXPECT_NEAR(foot.z(), -kMinReachM, 1e-4)
        << "这组角应把腿收到最紧，足端落在最小可达半径处";
}

// ---------------------------------------------------------------------------
// 可达点：IK + FK 回代
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, ReachableTargetIsSolvedAccurately) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    // 250mm 落在可达区间 [190.6, 310.6] 内部
    const SolveOutcome outcome = solve(kinematics, Eigen::Vector3d(0.0, 0.0, -0.25));

    EXPECT_GE(outcome.code, 0) << "可达点不应返回错误码";
    EXPECT_LT(outcome.error_norm, 1e-6)
        << "可达点的 FK 回代误差应接近 0，实际 = " << outcome.error_norm * 1000.0 << " mm";
}

TEST(LegKinematicsTest, TargetsJustInsideWorkspaceAreReachable) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    // 比最小可达半径大 10mm
    const SolveOutcome outcome = solve(kinematics, Eigen::Vector3d(0.0, 0.0, -(kMinReachM + 0.01)));

    EXPECT_GE(outcome.code, 0);
    EXPECT_LT(outcome.error_norm, 1e-6);
}

// ---------------------------------------------------------------------------
// 不可达点：求解器不会说"无解"，而是落到工作空间边界
//
// 这是最容易被忽视的坑：KDL 会返回一个关节向量，调用方如果只看
// "有没有返回结果"，就会把"够不着"当成"解出来了"。
// 所以必须用 FK 回代误差来判断。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, TargetInsideMinReachLandsOnWorkspaceBoundary) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    // 120mm 比最小可达半径 190.6mm 还小，物理上够不着
    const Eigen::Vector3d target(0.0, 0.0, -0.12);
    const SolveOutcome outcome = solve(kinematics, target);

    EXPECT_LT(outcome.code, 0) << "不可达目标应返回错误码（demo 链上通常是 -101）";

    // 求解器会把腿收到最紧、朝正下方，也就是落在最小可达半径上
    EXPECT_NEAR(outcome.achieved.norm(), kMinReachM, 1e-4);

    // 误差 = 190.6 - 120 = 70.6 mm。这个数字在真实的调参过程中出现过，
    // 它同时也是"目标点不可达"的定量证据。
    EXPECT_NEAR(outcome.error_norm, kMinReachM - 0.12, 1e-3);
}

TEST(LegKinematicsTest, TargetsInsideTheShellAreNeverReportedAsAccurate) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    // 球壳内部的一系列点：全部不可达，且误差都应等于"到最近边界的距离"
    for (const double radius : {0.05, 0.10, 0.15, 0.18}) {
        const SolveOutcome outcome = solve(kinematics, Eigen::Vector3d(0.0, 0.0, -radius));

        EXPECT_GT(outcome.error_norm, 1e-3)
            << "r=" << radius << " 位于球壳内部，本次却报出了很小的回代误差";
        EXPECT_NEAR(outcome.error_norm, kMinReachM - radius, 1e-3)
            << "r=" << radius << " 的误差应等于到最小可达半径的距离";
    }
}

// ---------------------------------------------------------------------------
// 位置偏移必须成对使用
//
// position_offset_ 在 IK 里加上、在 FK 里减掉。只要这一对是对称的，
// 设置非零偏移之后 FK(IK(p)) 仍然应该回到 API 坐标下的 p。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, PositionOffsetIsSymmetricBetweenIkAndFk) {
    LegKinematics kinematics(leg_calc::build_demo_chain());
    const Eigen::Vector3d offset(0.0, 0.0, -0.01);
    kinematics.set_position_offset(offset);

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
    LegKinematics kinematics(leg_calc::build_demo_chain());

    // 一个远离奇异的一般构型
    const JointVector joints = (JointVector() << 0.10, 1.20, 0.60).finished();
    const JointVector joint_vel(0.10, -0.20, 0.30);

    const Eigen::Vector3d foot_vel = kinematics.forward_velocity(joints, joint_vel);
    const JointVector recovered = kinematics.inverse_velocity(joints, foot_vel);

    EXPECT_LT((recovered - joint_vel).norm(), 1e-9)
        << "非奇异构型下 v -> q_dot 的映射应可逆";
}

// ---------------------------------------------------------------------------
// 径向可达区间：把"球壳判据"从测试搬进运行路径
//
// 预判的价值是省掉一次注定失败的数值迭代（LMA 最多 150 次 × 6 条腿）。
// 但它只是**必要条件**，所以解是否可信最终仍以 FK 回代误差为准。
// ---------------------------------------------------------------------------
TEST(LegKinematicsTest, ReachLimitsGateTheSolver) {
    LegKinematics kinematics(leg_calc::build_demo_chain());
    kinematics.set_reach_limits(leg_calc::kDemoChainMinReachM, leg_calc::kDemoChainMaxReachM);

    // 球壳内部（够不着）
    EXPECT_FALSE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.12)));
    // 球壳外部（伸不到）
    EXPECT_FALSE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.35)));
    // 球壳内部（可达）
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.25)));
    // 边界（闭区间，算可达）
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -leg_calc::kDemoChainMinReachM)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -leg_calc::kDemoChainMaxReachM)));
}

// 可达半径是**斜边长度**，与方向无关
TEST(LegKinematicsTest, ReachCheckIsDirectionIndependent) {
    LegKinematics kinematics(leg_calc::build_demo_chain());
    kinematics.set_reach_limits(leg_calc::kDemoChainMinReachM, leg_calc::kDemoChainMaxReachM);

    const double radius = 0.25;
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -radius)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(radius, 0.0, 0.0)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, radius, 0.0)));
}

// 预判必须和 inverse_position 用同一个坐标约定：可达性针对"加上 offset 之后"的目标
TEST(LegKinematicsTest, ReachCheckAccountsForPositionOffset) {
    LegKinematics kinematics(leg_calc::build_demo_chain());
    kinematics.set_reach_limits(leg_calc::kDemoChainMinReachM, leg_calc::kDemoChainMaxReachM);
    kinematics.set_position_offset(Eigen::Vector3d(0.0, 0.0, -0.01));

    // API 坐标 0.24 + offset 0.01 = 内部目标 0.25，可达
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.24)));
    // 0.12 + 0.01 = 0.13，仍然在球壳内，不可达
    EXPECT_FALSE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.12)));
}

// 默认不启用预判：任何点都算"在范围内"，行为与加预判之前一致
TEST(LegKinematicsTest, ReachCheckIsDisabledByDefault) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(0.0, 0.0, -0.01)));
    EXPECT_TRUE(kinematics.is_within_reach(Eigen::Vector3d(10.0, 0.0, 0.0)));
}

// ---------------------------------------------------------------------------
// demo_chain.hpp 里的可达区间常量必须与链的实际几何一致
//
// 做法是密集采样：在关节空间上撒点，用 FK 算出 |p| 的最小 / 最大值。
// 采样精度足以把误差压到 1e-5 量级，所以一旦有人改了链里的杆长却忘了
// 同步常量，这条断言会立刻失败。
// ---------------------------------------------------------------------------
TEST(DemoChainTest, ReachConstantsMatchTheActualChain) {
    LegKinematics kinematics(leg_calc::build_demo_chain());

    double min_radius = 1e9;
    double max_radius = 0.0;
    constexpr int kQ1Samples = 64;
    constexpr int kQ2Samples = 2000;

    for (int i = 0; i < kQ1Samples; ++i) {
        const double q1 = 2.0 * kPi * static_cast<double>(i) / kQ1Samples;
        for (int j = 0; j < kQ2Samples; ++j) {
            const double q2 = 2.0 * kPi * static_cast<double>(j) / kQ2Samples;
            const JointVector joints = (JointVector() << 0.0, q1, q2).finished();
            const double radius = kinematics.forward_position(joints).norm();
            min_radius = std::min(min_radius, radius);
            max_radius = std::max(max_radius, radius);
        }
    }

    // 采样只会"够不到"极值（采样值 >= 真实最小值、<= 真实最大值），
    // 所以用 1e-5 的容差就足以判定常量是否写对。
    EXPECT_NEAR(min_radius, leg_calc::kDemoChainMinReachM, 1e-5)
        << "最小可达半径与 demo_chain.hpp 里的常量不一致：改了链的杆长就要重算常量";
    EXPECT_NEAR(max_radius, leg_calc::kDemoChainMaxReachM, 1e-5)
        << "最大可达半径与 demo_chain.hpp 里的常量不一致";
}
