// FootTrajectory 单元测试
//
// 这些断言把"步态层应该满足的物理约束"固定下来。它们全部来自实际调试中
// 踩过的坑，所以每一条失败都对应一个真实发生过的错误：
//
//   1. 支撑相方向写反  -> 足端在支撑相里向前移，每个相位切换点跳一个整步长
//   2. 速度/步长量纲错 -> 步长与命令速度脱钩（曾出现命令 0.2m/s 实走 0.016m/s）
//   3. 缺少上限夹取    -> 外部速度命令可以超出腿的机械能力
//
// 运行：colcon test --packages-select leg_calc

#include <gtest/gtest.h>

#include <cmath>

#include "leg_calc/foot_trajectory.hpp"

namespace {

using leg_calc::BodyTwist;
using leg_calc::FootTrajectory;
using leg_calc::GaitConfig;
using leg_calc::GaitPattern;
using leg_calc::LegId;
using leg_calc::LegPhase;

// 基准配置。三个关键换算先手算清楚，后面所有断言都基于它们：
//   frequency_hz = 1.0  -> 步态周期 1.0 s
//   Tripod              -> stance_duty = 1/2，支撑相时长 = 0.5 s
//   step_length_m = 0.04
// -> vx = 0.08 m/s 时步长 = 0.08 × 0.5 = 0.04 m，正好跑满上限
constexpr double kFrequencyHz = 1.0;
constexpr double kStanceDurationS = 0.5;
constexpr double kStepLimitM = 0.04;
constexpr double kLateralLimitM = 0.02;
constexpr double kTurnLimitRad = 0.15;
constexpr double kStepHeightM = 0.03;
constexpr double kBodyHeightM = 0.25;

GaitConfig make_config() {
    GaitConfig config;
    config.pattern = GaitPattern::Tripod;
    config.frequency_hz = kFrequencyHz;
    config.step_length_m = kStepLimitM;
    config.step_height_m = kStepHeightM;
    config.lateral_step_m = kLateralLimitM;
    config.turn_step_rad = kTurnLimitRad;
    config.body_height_m = kBodyHeightM;
    return config;
}

// 站姿下左前腿的足端名义位置（身体坐标系）
Eigen::Vector3d nominal_foot() {
    return Eigen::Vector3d(0.18, 0.12, -kBodyHeightM);
}

BodyTwist linear_twist(double vx, double vy) {
    BodyTwist twist;
    twist.linear = Eigen::Vector3d(vx, vy, 0.0);
    return twist;
}

BodyTwist forward_twist(double vx) {
    return linear_twist(vx, 0.0);
}

Eigen::Vector3d foot_at(
    FootTrajectory& traj,
    const Eigen::Vector3d& nominal,
    LegPhase phase,
    double fraction,
    const BodyTwist& twist) {
    return traj.compute_foot_target(LegId::LeftFront, nominal, phase, fraction, twist);
}

// 一个支撑相内足端相对身体的移动量（正值 = 向后退了这么多，单位 m）
double stance_travel_x(
    FootTrajectory& traj,
    const Eigen::Vector3d& nominal,
    const BodyTwist& twist) {
    const auto start = foot_at(traj, nominal, LegPhase::Stance, 0.0, twist);
    const auto end = foot_at(traj, nominal, LegPhase::Stance, 1.0, twist);
    return start.x() - end.x();
}

}  // namespace

// ---------------------------------------------------------------------------
// 支撑相方向
//
// 物理前提：脚掌踩在地上、在世界坐标系里保持不动。身体向前走，
// 所以站在身体坐标系里看，足端必须"向后退"（x 递减）。
//
// 这一条曾经写反过：足端在支撑相里向前移，结果是每个相位切换点都出现
// 一个整步长的瞬跳（实测 38.4mm），而且脚在地上打滑、推不动身体。
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, StanceMovesFootBackward) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    const auto twist = forward_twist(0.08);

    const auto at_start = foot_at(traj, nominal, LegPhase::Stance, 0.0, twist);
    const auto at_end = foot_at(traj, nominal, LegPhase::Stance, 1.0, twist);

    EXPECT_GT(at_start.x(), at_end.x())
        << "支撑相足端必须向后移动（x 递减）：脚踩地不动，身体前进";
}

// 支撑相的总行程必须等于一个完整步长
TEST(FootTrajectoryTest, StanceTravelEqualsStepLength) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    // 0.08 m/s × 0.5 s = 0.04 m，正好等于上限
    const auto twist = forward_twist(0.08);

    EXPECT_NEAR(stance_travel_x(traj, nominal, twist), kStepLimitM, 1e-12);
}

// ---------------------------------------------------------------------------
// 摆动相方向
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, SwingMovesFootForward) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    const auto twist = forward_twist(0.08);

    const auto at_start = foot_at(traj, nominal, LegPhase::Swing, 0.0, twist);
    const auto at_end = foot_at(traj, nominal, LegPhase::Swing, 1.0, twist);

    EXPECT_LT(at_start.x(), at_end.x())
        << "摆动相足端必须向前摆（x 递增）";
}

// 摆动相抬腿：起止点在地面，中点到最高
TEST(FootTrajectoryTest, SwingLiftReachesStepHeightAtMidPhase) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    const auto twist = forward_twist(0.08);

    EXPECT_NEAR(foot_at(traj, nominal, LegPhase::Swing, 0.0, twist).z(), nominal.z(), 1e-12);
    EXPECT_NEAR(foot_at(traj, nominal, LegPhase::Swing, 0.5, twist).z(), nominal.z() + kStepHeightM, 1e-12);
    EXPECT_NEAR(foot_at(traj, nominal, LegPhase::Swing, 1.0, twist).z(), nominal.z(), 1e-12);
}

// 抬腿高度由相位决定，与速度命令无关（原地踏步也要抬脚）
TEST(FootTrajectoryTest, SwingLiftDoesNotDependOnVelocity) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();

    const double z_rest = foot_at(traj, nominal, LegPhase::Swing, 0.5, linear_twist(0.0, 0.0)).z();
    const double z_move = foot_at(traj, nominal, LegPhase::Swing, 0.5, forward_twist(0.08)).z();

    EXPECT_NEAR(z_rest, z_move, 1e-12);
    EXPECT_NEAR(z_rest, nominal.z() + kStepHeightM, 1e-12);
}

// ---------------------------------------------------------------------------
// 两相衔接
//
// 支撑相结束的位置必须等于摆动相开始的位置，反之亦然。
// 否则每个相位切换点都会出现位置瞬跳。这条断言一旦失败，
// 说明支撑相/摆动相的运动方向或符号至少有一个错了。
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, PhasesJoinContinuously) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    const auto twist = forward_twist(0.08);

    const auto stance_start = foot_at(traj, nominal, LegPhase::Stance, 0.0, twist);
    const auto stance_end = foot_at(traj, nominal, LegPhase::Stance, 1.0, twist);
    const auto swing_start = foot_at(traj, nominal, LegPhase::Swing, 0.0, twist);
    const auto swing_end = foot_at(traj, nominal, LegPhase::Swing, 1.0, twist);

    EXPECT_TRUE(stance_end.isApprox(swing_start, 1e-12))
        << "支撑相结束位置必须等于摆动相开始位置";
    EXPECT_TRUE(swing_end.isApprox(stance_start, 1e-12))
        << "摆动相结束位置必须等于支撑相开始位置";
}

// ---------------------------------------------------------------------------
// 速度 -> 步长的换算
//
// 唯一正确的物理关系是：步长 = 速度 × 支撑相时长。
// 曾经写成 `vx × step_half`（m/s × m = m²/s，量纲不成立），
// 导致命令速度和实际步长完全脱钩。
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, StepLengthScalesWithVelocity) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();

    // 0.04 m/s × 0.5 s = 20mm；0.02 m/s × 0.5 s = 10mm（都未触及 40mm 上限）
    EXPECT_NEAR(stance_travel_x(traj, nominal, forward_twist(0.04)), 0.02, 1e-12);
    EXPECT_NEAR(stance_travel_x(traj, nominal, forward_twist(0.02)), 0.01, 1e-12);
    EXPECT_NEAR(
        stance_travel_x(traj, nominal, forward_twist(0.04)),
        2.0 * stance_travel_x(traj, nominal, forward_twist(0.02)),
        1e-12);
}

TEST(FootTrajectoryTest, StepLengthScalesInverselyWithFrequency) {
    auto slow_config = make_config();  // 1.0 Hz -> 支撑相 0.5 s
    auto fast_config = make_config();
    fast_config.frequency_hz = 2.0;  // 2.0 Hz -> 支撑相 0.25 s

    FootTrajectory slow(slow_config);
    FootTrajectory fast(fast_config);
    const auto nominal = nominal_foot();
    const auto twist = forward_twist(0.04);

    EXPECT_NEAR(stance_travel_x(slow, nominal, twist), 0.02, 1e-12);
    EXPECT_NEAR(stance_travel_x(fast, nominal, twist), 0.01, 1e-12);
}

// 支撑相占比取决于步态结构，不能统一写死 0.5
TEST(FootTrajectoryTest, StepLengthDependsOnGaitPattern) {
    auto tripod_config = make_config();  // stance_duty = 1/2
    auto wave_config = make_config();
    wave_config.pattern = GaitPattern::Wave;  // stance_duty = 5/6

    FootTrajectory tripod(tripod_config);
    FootTrajectory wave(wave_config);
    const auto nominal = nominal_foot();
    const auto twist = forward_twist(0.01);  // 小速度，远离上限

    const double tripod_step = stance_travel_x(tripod, nominal, twist);
    const double wave_step = stance_travel_x(wave, nominal, twist);

    EXPECT_NEAR(tripod_step, 0.01 * 0.5, 1e-12);
    EXPECT_NEAR(wave_step, 0.01 * (5.0 / 6.0), 1e-12);
    EXPECT_NEAR(wave_step / tripod_step, 5.0 / 3.0, 1e-12);
}

// ---------------------------------------------------------------------------
// 上限夹取
//
// vx/vy/wz 来自外部输入，可能远超腿的机械能力；轨迹层必须在算完之后
// 夹到 GaitConfig 的上限，而不是让 IK 去解一个不可能的目标。
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, StepLengthIsClampedToConfigLimit) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();

    // 10 m/s 显然超出机械能力，步长应被夹到 +step_length_m
    EXPECT_NEAR(stance_travel_x(traj, nominal, forward_twist(10.0)), kStepLimitM, 1e-12);

    // 反向命令：身体后退 => 脚相对身体向前移，所以还是递减方向，
    // 但"向后退的距离"这个量本身取负，被夹到 -step_length_m。
    EXPECT_NEAR(stance_travel_x(traj, nominal, forward_twist(-10.0)), -kStepLimitM, 1e-12);
}

TEST(FootTrajectoryTest, LateralStepIsClampedToLateralLimit) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    const auto twist = linear_twist(0.0, 10.0);

    const auto start = foot_at(traj, nominal, LegPhase::Stance, 0.0, twist);
    const auto end = foot_at(traj, nominal, LegPhase::Stance, 1.0, twist);

    // 横向也必须是"向后退"的方向（y 递减）
    EXPECT_GT(start.y(), end.y());
    EXPECT_NEAR(start.y() - end.y(), kLateralLimitM, 1e-12);
}

// ---------------------------------------------------------------------------
// 零命令
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, ZeroCommandLeavesPlanarPositionUnchanged) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();
    const BodyTwist zero;

    for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        const auto stance = foot_at(traj, nominal, LegPhase::Stance, fraction, zero);
        EXPECT_NEAR(stance.x(), nominal.x(), 1e-12);
        EXPECT_NEAR(stance.y(), nominal.y(), 1e-12);
        EXPECT_NEAR(stance.z(), nominal.z(), 1e-12);

        // 摆动相只允许 z 抬升，水平位置不动
        const auto swing = foot_at(traj, nominal, LegPhase::Swing, fraction, zero);
        EXPECT_NEAR(swing.x(), nominal.x(), 1e-12);
        EXPECT_NEAR(swing.y(), nominal.y(), 1e-12);
    }
}

// ---------------------------------------------------------------------------
// 转向
//
// wz 让足端绕身体 z 轴转动。支撑相与摆动相的转角方向相反，
// 才能满足衔接条件。注意 apply_planar_turn 目前是一阶小角近似，
// 所以这里用"角度增量"来断言，而不是断言精确坐标。
// ---------------------------------------------------------------------------
TEST(FootTrajectoryTest, TurnRotatesFootAroundBodyZAxis) {
    FootTrajectory traj(make_config());
    const auto nominal = nominal_foot();

    BodyTwist twist;
    // 0.3 rad/s × 0.5 s = 0.15 rad，正好等于 turn_step_rad 上限
    twist.angular = Eigen::Vector3d(0.0, 0.0, 0.3);

    const auto rotated = foot_at(traj, nominal, LegPhase::Stance, 0.0, twist);

    // 支撑相起点施加 0.5 × 0.15 = 0.075 rad
    const double angle_before = std::atan2(nominal.y(), nominal.x());
    const double angle_after = std::atan2(rotated.y(), rotated.x());
    EXPECT_NEAR(angle_after - angle_before, 0.075, 1e-3)
        << "wz > 0 时足端应绕身体 z 轴逆时针转动";

    // 零角速度不产生任何转动
    const auto untouched = foot_at(traj, nominal, LegPhase::Stance, 0.0, forward_twist(0.0));
    EXPECT_NEAR(untouched.x(), nominal.x(), 1e-12);
    EXPECT_NEAR(untouched.y(), nominal.y(), 1e-12);
}
