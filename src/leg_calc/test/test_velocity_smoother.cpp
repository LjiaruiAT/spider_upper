// 速度命令平滑器测试
//
// 它补的是"运行中命令突变"这一块：轨迹层的位移 = 速度 × 支撑相时长，
// 这个式子是瞬时成立的，所以速度一跳、步长就跟着跳。这些用例锁住三件事：
//
//   1. 单步变化量确实受加速度上限约束 —— 限幅真的在起作用
//   2. 不过冲、不抖动、按理论时间到达 —— 限幅没有引入新的坏行为
//   3. 上限为 0 时能一键退回旧行为 —— 对照实验的开关还在
//
// 运行：colcon test --packages-select leg_calc

#include <gtest/gtest.h>

#include "leg_calc/gait_types.hpp"
#include "leg_calc/velocity_smoother.hpp"

namespace {

using leg_calc::BodyTwist;
using leg_calc::GaitConfig;
using leg_calc::VelocitySmoother;

// 与控制循环的默认周期（control_period_ms = 20）保持一致
constexpr double kDt = 0.02;

GaitConfig make_config(double max_linear_accel_mps2, double max_angular_accel_rps2) {
    GaitConfig config;
    config.max_linear_accel_mps2 = max_linear_accel_mps2;
    config.max_angular_accel_rps2 = max_angular_accel_rps2;
    return config;
}

BodyTwist planar(double vx, double vy, double wz) {
    BodyTwist twist;
    twist.linear = Eigen::Vector3d(vx, vy, 0.0);
    twist.angular = Eigen::Vector3d(0.0, 0.0, wz);
    return twist;
}

}  // namespace

// ---------------------------------------------------------------------------
// 限幅：单步变化量不能超过 加速度上限 × dt
// ---------------------------------------------------------------------------
TEST(VelocitySmootherTest, RampUpIsLimitedByAcceleration) {
    VelocitySmoother smoother(make_config(0.4, 1.0));
    const BodyTwist target = planar(0.2, 0.0, 0.0);

    const double max_step = 0.4 * kDt;  // 0.008 m/s
    double previous = 0.0;
    for (int i = 0; i < 50; ++i) {
        const BodyTwist now = smoother.update(target, kDt);
        const double step = now.linear.x() - previous;
        EXPECT_LE(step, max_step + 1e-12) << "第 " << i << " 步超出加速度上限";
        EXPECT_GE(step, 0.0) << "第 " << i << " 步方向反了";
        previous = now.linear.x();
    }
}

TEST(VelocitySmootherTest, ReachesTargetInExpectedTime) {
    // 0.2 m/s ÷ 0.4 m/s² = 0.5 s = 25 个控制周期
    VelocitySmoother slower(make_config(0.4, 1.0));
    const BodyTwist target = planar(0.2, 0.0, 0.0);

    for (int i = 0; i < 27; ++i) {  // 25 + 2 步余量，避开浮点边界
        slower.update(target, kDt);
    }
    EXPECT_DOUBLE_EQ(slower.current().linear.x(), 0.2) << "应该已经到达目标";

    // 反面：不该比理论时间更快到达
    VelocitySmoother faster(make_config(0.4, 1.0));
    for (int i = 0; i < 24; ++i) {
        faster.update(target, kDt);
    }
    EXPECT_LT(faster.current().linear.x(), 0.2) << "24 步（0.48s）不该已经到满速";
}

TEST(VelocitySmootherTest, DoesNotOvershootTarget) {
    VelocitySmoother smoother(make_config(0.4, 1.0));
    const BodyTwist target = planar(0.2, -0.1, 0.5);

    for (int i = 0; i < 300; ++i) {
        const BodyTwist now = smoother.update(target, kDt);
        EXPECT_LE(now.linear.x(), target.linear.x() + 1e-15) << "第 " << i << " 步过冲";
        EXPECT_GE(now.linear.y(), target.linear.y() - 1e-15) << "第 " << i << " 步过冲";
        EXPECT_LE(now.angular.z(), target.angular.z() + 1e-15) << "第 " << i << " 步过冲";
    }
}

// ---------------------------------------------------------------------------
// 反转：本类存在的理由
//
// 命令 0.2 → −0.2 时，如果不平滑，步长会在一个控制周期内从 +40mm 跳到 −40mm。
// 这里验证翻转过程被拉长到 1.0 s（0.4 m/s ÷ 0.4 m/s²）。
// ---------------------------------------------------------------------------
TEST(VelocitySmootherTest, ReversalIsSmooth) {
    VelocitySmoother smoother(make_config(0.4, 1.0));
    for (int i = 0; i < 60; ++i) {
        smoother.update(planar(0.2, 0.0, 0.0), kDt);
    }
    ASSERT_DOUBLE_EQ(smoother.current().linear.x(), 0.2);

    const BodyTwist reversed = planar(-0.2, 0.0, 0.0);
    const double max_step = 0.4 * kDt;
    double previous = smoother.current().linear.x();
    for (int i = 0; i < 100; ++i) {
        const BodyTwist now = smoother.update(reversed, kDt);
        const double step = now.linear.x() - previous;
        EXPECT_LE(step, 0.0) << "翻转过程应单调下降，第 " << i << " 步反向了";
        EXPECT_GE(step, -max_step - 1e-12) << "第 " << i << " 步变化过快";
        previous = now.linear.x();
    }
    EXPECT_NEAR(smoother.current().linear.x(), -0.2, 1e-12);

    // 0.98 s（49 步）内不该完成 0.4 m/s 的翻转
    VelocitySmoother no_time_travel(make_config(0.4, 1.0));
    for (int i = 0; i < 60; ++i) {
        no_time_travel.update(planar(0.2, 0.0, 0.0), kDt);
    }
    for (int i = 0; i < 49; ++i) {
        no_time_travel.update(reversed, kDt);
    }
    EXPECT_GT(no_time_travel.current().linear.x(), -0.2);
}

// ---------------------------------------------------------------------------
// 三个分量各自独立、各用各的上限
// ---------------------------------------------------------------------------
TEST(VelocitySmootherTest, LateralAxisIsSmoothedToo) {
    VelocitySmoother smoother(make_config(0.4, 1.0));
    const BodyTwist now = smoother.update(planar(0.0, -0.1, 0.0), kDt);
    EXPECT_NEAR(now.linear.y(), -0.4 * kDt, 1e-12) << "横移也必须被限幅";
}

TEST(VelocitySmootherTest, AngularUsesItsOwnLimit) {
    // 角速度用另一个上限：1.0 rad/s² × 0.02 s = 0.02 rad/s
    VelocitySmoother smoother(make_config(0.4, 1.0));
    const BodyTwist now = smoother.update(planar(0.0, 0.0, 0.5), kDt);
    EXPECT_NEAR(now.angular.z(), 1.0 * kDt, 1e-12);
}

// ---------------------------------------------------------------------------
// 边界与开关
// ---------------------------------------------------------------------------
TEST(VelocitySmootherTest, ZeroDtDoesNotChangeVelocity) {
    VelocitySmoother smoother(make_config(0.4, 1.0));
    smoother.update(planar(0.2, 0.0, 0.0), kDt);

    const double before = smoother.current().linear.x();
    const BodyTwist after = smoother.update(planar(0.2, 0.0, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(after.linear.x(), before)
        << "dt = 0 不该绕过限幅直接跳到目标速度";
}

TEST(VelocitySmootherTest, ZeroAccelLimitFollowsTargetImmediately) {
    // 上限为 0 = 关闭平滑，回到"命令即输出"的旧行为
    VelocitySmoother smoother(make_config(0.0, 0.0));
    const BodyTwist now = smoother.update(planar(0.2, -0.1, 0.5), kDt);
    EXPECT_DOUBLE_EQ(now.linear.x(), 0.2);
    EXPECT_DOUBLE_EQ(now.linear.y(), -0.1);
    EXPECT_DOUBLE_EQ(now.angular.z(), 0.5);
}

TEST(VelocitySmootherTest, ResetReturnsToZero) {
    VelocitySmoother smoother(make_config(0.4, 1.0));
    for (int i = 0; i < 40; ++i) {
        smoother.update(planar(0.2, 0.1, 0.3), kDt);
    }
    ASSERT_GT(smoother.current().linear.x(), 0.0)
        << "前置条件：平滑器应该已经积累出非零速度";

    smoother.reset();
    EXPECT_DOUBLE_EQ(smoother.current().linear.x(), 0.0);
    EXPECT_DOUBLE_EQ(smoother.current().linear.y(), 0.0);
    EXPECT_DOUBLE_EQ(smoother.current().angular.z(), 0.0);
}

// ---------------------------------------------------------------------------
// 默认配置必须真的启用平滑
//
// 这条防的是"参数默认值被填成 0"——那样这个类会静默退化成恒等变换，
// 编译通过、测试全绿，但功能等于没接上。
// ---------------------------------------------------------------------------
TEST(VelocitySmootherTest, DefaultConfigActuallySmooths) {
    const GaitConfig config;
    EXPECT_GT(config.max_linear_accel_mps2, 0.0);
    EXPECT_GT(config.max_angular_accel_rps2, 0.0);

    VelocitySmoother smoother(config);
    const BodyTwist now = smoother.update(planar(0.2, 0.0, 0.0), kDt);
    EXPECT_LT(now.linear.x(), 0.2) << "默认配置下第一步不应该直接到达目标速度";
}
