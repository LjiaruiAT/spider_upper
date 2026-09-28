// OdometryIntegrator 测试
//
// 这个类只影响"RViz 里看到什么"，不影响机器人动作，所以它的 bug **不会造成危险**
// ——但会让可视化变成谎言。一个悄悄发散的里程计会让你把"显示在飘"误判成
// "步态有问题"，然后去改本来没错的东西。
//
// 所以测试重点有两个：
//   ① 直行、横移、原地转这些"应该精确"的情况必须精确（它们是欧拉积分也不会错的）
//   ② **转弯必须能精确闭合**——这是圆弧积分相对显式欧拉的唯一可测量差别，
//      也是这个类唯一的"数学"部分

#include <gtest/gtest.h>

#include <cmath>

#include "leg_calc/odometry_integrator.hpp"

using leg_calc::BodyTwist;
using leg_calc::OdometryIntegrator;

namespace {

constexpr double kDt = 0.02;  // 与节点默认控制周期一致

BodyTwist make_twist(double vx, double vy, double wz) {
    BodyTwist twist;
    twist.linear = Eigen::Vector3d(vx, vy, 0.0);
    twist.angular = Eigen::Vector3d(0.0, 0.0, wz);
    return twist;
}

// 走 steps 步同样的速度。
void run(OdometryIntegrator& odom, double vx, double vy, double wz, int steps) {
    const auto twist = make_twist(vx, vy, wz);
    for (int i = 0; i < steps; ++i) {
        odom.update(twist, kDt);
    }
}

// 从旋转矩阵里取航向角（会绕回 [-π, π]）。
double yaw_of(const OdometryIntegrator& odom) {
    const auto& r = odom.odom_T_base().linear();
    return std::atan2(r(1, 0), r(0, 0));
}

}  // namespace

TEST(OdometryIntegratorTest, StartsAtIdentity) {
    OdometryIntegrator odom;
    EXPECT_TRUE(odom.odom_T_base().matrix().isApprox(Eigen::Matrix4d::Identity()));
}

// 零速度不该改变任何东西。看着显然，但如果实现里漏了 early return、
// 或者 ICR 分支在 w=0 时算出了 NaN，这条会立刻炸出来。
TEST(OdometryIntegratorTest, ZeroTwistKeepsPose) {
    OdometryIntegrator odom;
    run(odom, 0.0, 0.0, 0.0, 500);
    EXPECT_TRUE(odom.odom_T_base().matrix().isApprox(Eigen::Matrix4d::Identity()));
}

// 直行走的是"纯平移"分支，必须**精确**（没有积分误差）。
TEST(OdometryIntegratorTest, StraightLineIsExact) {
    OdometryIntegrator odom;
    run(odom, 0.2, 0.0, 0.0, 50);  // 50 × 20ms = 1.0s
    const auto& t = odom.odom_T_base().translation();
    EXPECT_NEAR(t.x(), 0.2, 1e-12);
    EXPECT_NEAR(t.y(), 0.0, 1e-12);
    EXPECT_NEAR(yaw_of(odom), 0.0, 1e-12);
}

// 横移同样走纯平移分支。
TEST(OdometryIntegratorTest, LateralStrafeIsExact) {
    OdometryIntegrator odom;
    run(odom, 0.0, 0.1, 0.0, 100);  // 2.0s
    const auto& t = odom.odom_T_base().translation();
    EXPECT_NEAR(t.x(), 0.0, 1e-12);
    EXPECT_NEAR(t.y(), 0.2, 1e-12);
}

// 原地转：v = 0 → 瞬时旋转中心就是本体原点，位置必须一点不动。
// 这条能拦住"ICR 公式符号写反"这类错误（写反了位置会画出一个圆）。
TEST(OdometryIntegratorTest, RotateInPlaceKeepsPosition) {
    OdometryIntegrator odom;
    run(odom, 0.0, 0.0, 0.5, 100);  // 0.5 rad/s × 2.0s = 1.0 rad
    const auto& t = odom.odom_T_base().translation();
    EXPECT_NEAR(t.x(), 0.0, 1e-12);
    EXPECT_NEAR(t.y(), 0.0, 1e-12);
    EXPECT_NEAR(yaw_of(odom), 1.0, 1e-12);
}

// 转 90° 之后再前进，应该沿**新的**朝向走，也就是里程计系的 +y。
// 这条验证的是"右乘"这个顺序对不对——写错成左乘就会沿旧朝向走。
TEST(OdometryIntegratorTest, TranslationFollowsCurrentHeading) {
    OdometryIntegrator odom;
    const double target = M_PI / 2.0;
    run(odom, 0.0, 0.0, target, 50);  // 1.0s 转 90°
    run(odom, 0.1, 0.0, 0.0, 50);     // 1.0s 前进 0.1m

    const auto& t = odom.odom_T_base().translation();
    EXPECT_NEAR(t.x(), 0.0, 1e-9);
    EXPECT_NEAR(t.y(), 0.1, 1e-9);
    EXPECT_NEAR(yaw_of(odom), target, 1e-12);
}

// ★ 核心用例：匀速转圈必须**精确闭合**。
//
// 这是圆弧积分相对显式欧拉的唯一可测量差别：
// 显式欧拉每圈的半径会外扩 exp(π·w·dt) —— 以 w=0.314、dt=20ms 算约 +2%，
// 转几圈就明显飘。圆弧积分对匀速输入是解析解，应该只在浮点精度内闭合。
TEST(OdometryIntegratorTest, ConstantTwistCircleClosesExactly) {
    constexpr double kLinear = 0.2;
    constexpr int kSteps = 1000;  // 20.0 s
    const double angular = 2.0 * M_PI / (kSteps * kDt);  // 正好一圈

    OdometryIntegrator odom;
    run(odom, kLinear, 0.0, angular, kSteps);

    const auto& t = odom.odom_T_base().translation();
    EXPECT_NEAR(t.x(), 0.0, 1e-9);
    EXPECT_NEAR(t.y(), 0.0, 1e-9);
    // 走完一整圈，航向也应该回到 0（绕回后 2π ≡ 0）
    EXPECT_NEAR(yaw_of(odom), 0.0, 1e-9);
}

// 转圈过程中，到圆心的距离必须**恒定**等于 R = v / w，
// 而且圆心在出发朝向的左侧（vx>0、w>0 时就在 +y 方向）。
// 闭合成不闭合只说明"总位移对不对"，这条说明"中间也没飘"。
TEST(OdometryIntegratorTest, CircleRadiusStaysConstant) {
    constexpr double kLinear = 0.2;
    constexpr int kSteps = 1000;
    const double angular = 2.0 * M_PI / (kSteps * kDt);
    const double radius = kLinear / angular;  // R = v / w

    OdometryIntegrator odom;
    const auto twist = make_twist(kLinear, 0.0, angular);
    double worst = 0.0;
    for (int i = 0; i < kSteps; ++i) {
        odom.update(twist, kDt);
        const auto& t = odom.odom_T_base().translation();
        // 圆心 (0, R)：左转时瞬时旋转中心在行进方向的左侧
        const double d = std::hypot(t.x(), t.y() - radius);
        worst = std::max(worst, std::fabs(d - radius));
    }
    EXPECT_LT(worst, 1e-9);
}

// 一圈没走完也应该留在圆上（防止"只有整圈才闭合"这种巧合式的通过）。
TEST(OdometryIntegratorTest, HalfCircleLandsOppositeSide) {
    constexpr double kLinear = 0.2;
    constexpr int kSteps = 500;  // 半圈
    const double angular = 2.0 * M_PI / (1000 * kDt);
    const double radius = kLinear / angular;

    OdometryIntegrator odom;
    run(odom, kLinear, 0.0, angular, kSteps);

    // 从 (0,0) 绕圆心 (0,R) 转半圈 → 落在 (0, 2R)
    const auto& t = odom.odom_T_base().translation();
    EXPECT_NEAR(t.x(), 0.0, 1e-9);
    EXPECT_NEAR(t.y(), 2.0 * radius, 1e-9);
    // 航向转了 π
    EXPECT_NEAR(std::fabs(yaw_of(odom)), M_PI, 1e-9);
}

// dt <= 0 直接忽略。一次时钟异常不该把位姿搞成 NaN——
// 那会让 RViz 里整棵 TF 树消失，而且看不出跟时钟有关。
TEST(OdometryIntegratorTest, NonPositiveDtIsIgnored) {
    OdometryIntegrator odom;
    const auto twist = make_twist(0.2, 0.0, 0.0);
    odom.update(twist, 0.0);
    odom.update(twist, -0.02);
    EXPECT_TRUE(odom.odom_T_base().matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(OdometryIntegratorTest, ResetReturnsToIdentity) {
    OdometryIntegrator odom;
    run(odom, 0.2, 0.1, 0.3, 200);
    ASSERT_FALSE(odom.odom_T_base().matrix().isApprox(Eigen::Matrix4d::Identity()));

    odom.reset();
    EXPECT_TRUE(odom.odom_T_base().matrix().isApprox(Eigen::Matrix4d::Identity()));
}
