// gait_types 里的工具函数测试
//
// 这些函数看着简单，但每一个都被别处依赖，而且都容易"改坏了没人发现"：
//
//   stance_duty / stance_duration_s  —— 步长换算的分母
//   max_*_speed                      —— "命令速度是否超出能力"的判据
//   quintic_ease                     —— 起步/停步过渡的曲线
//
// 运行：colcon test --packages-select leg_calc

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "leg_calc/gait_types.hpp"

namespace {

using leg_calc::GaitConfig;
using leg_calc::GaitPattern;
using leg_calc::gait_pattern_name;
using leg_calc::max_forward_speed_mps;
using leg_calc::max_lateral_speed_mps;
using leg_calc::max_turn_rate_rps;
using leg_calc::parse_gait_pattern;
using leg_calc::quintic_ease;
using leg_calc::stance_duration_s;
using leg_calc::stance_duty;

GaitConfig make_config(GaitPattern pattern, double frequency_hz) {
    GaitConfig config;
    config.pattern = pattern;
    config.frequency_hz = frequency_hz;
    return config;
}

// 端点处用单侧差分：quintic_ease 在 [0,1] 之外被夹住，
// 双侧差分会被夹取本身影响，测不出多项式在端点处的斜率。
double one_sided_derivative_at_zero(double h) {
    return (quintic_ease(h) - quintic_ease(0.0)) / h;
}

double one_sided_derivative_at_one(double h) {
    return (quintic_ease(1.0) - quintic_ease(1.0 - h)) / h;
}

}  // namespace

// ---------------------------------------------------------------------------
// 步态名称 <-> 枚举
//
// 节点的 gait_pattern 参数与启动打印都走这两个函数。
// 解析必须严格（大小写敏感、非法值抛异常）——配置写错时宁可启动失败，
// 也不要静默退回 Tripod 却让人以为在跑 Wave。
// ---------------------------------------------------------------------------
TEST(GaitPatternNameTest, ParseAcceptsTheThreeNames) {
    EXPECT_EQ(parse_gait_pattern("tripod"), GaitPattern::Tripod);
    EXPECT_EQ(parse_gait_pattern("ripple"), GaitPattern::Ripple);
    EXPECT_EQ(parse_gait_pattern("wave"), GaitPattern::Wave);
}

TEST(GaitPatternNameTest, ParseRejectsUnknownNames) {
    for (const char* bad : {"", "Tripod", "WAVE", "walk", "0"}) {
        EXPECT_THROW(parse_gait_pattern(bad), std::invalid_argument) << "bad=" << bad;
    }
}

TEST(GaitPatternNameTest, NameRoundTripsThroughParse) {
    for (const auto pattern : {GaitPattern::Tripod, GaitPattern::Ripple, GaitPattern::Wave}) {
        EXPECT_EQ(parse_gait_pattern(gait_pattern_name(pattern)), pattern);
    }
}

// ---------------------------------------------------------------------------
// stance_duty：支撑相占整个步态周期的比例
// ---------------------------------------------------------------------------
TEST(GaitTypesTest, StanceDutyMatchesEachGaitStructure) {
    EXPECT_DOUBLE_EQ(stance_duty(GaitPattern::Tripod), 0.5);       // 两组各占一半
    EXPECT_DOUBLE_EQ(stance_duty(GaitPattern::Ripple), 2.0 / 3.0); // 三组，摆动各占 1/3
    EXPECT_DOUBLE_EQ(stance_duty(GaitPattern::Wave), 5.0 / 6.0);   // 单腿摆动，窗口 1/6
}

// ---------------------------------------------------------------------------
// stance_duration_s：一个支撑相真实持续多久
// ---------------------------------------------------------------------------
TEST(GaitTypesTest, StanceDurationFollowsFrequency) {
    // Tripod：周期 1s 时支撑相 0.5s
    EXPECT_NEAR(stance_duration_s(make_config(GaitPattern::Tripod, 1.0)), 0.5, 1e-12);
    // 频率加倍，支撑相减半
    EXPECT_NEAR(stance_duration_s(make_config(GaitPattern::Tripod, 2.0)), 0.25, 1e-12);
    EXPECT_NEAR(stance_duration_s(make_config(GaitPattern::Tripod, 2.5)), 0.2, 1e-12);
    // Wave：支撑相占 5/6 个周期
    EXPECT_NEAR(stance_duration_s(make_config(GaitPattern::Wave, 1.0)), 5.0 / 6.0, 1e-12);
}

TEST(GaitTypesTest, StanceDurationSurvivesInvalidFrequency) {
    // 频率为 0 或负数时不能出现 inf / nan（内部有下限保护）
    for (const double bad_frequency : {0.0, -1.0}) {
        const double duration = stance_duration_s(make_config(GaitPattern::Tripod, bad_frequency));
        EXPECT_TRUE(std::isfinite(duration)) << "frequency=" << bad_frequency;
        EXPECT_GT(duration, 0.0);
    }
}

// ---------------------------------------------------------------------------
// 能力上限：命令速度超过它就一定会被步长上限夹住
//
// 这是"参数互相耦合"的量化表达：
//   最大速度 = 步长上限 ÷ 支撑相时长
// 反过来验证：最大速度 × 支撑相时长 必须等于步长上限。
// ---------------------------------------------------------------------------
TEST(GaitTypesTest, MaxForwardSpeedIsConsistentWithStepLimit) {
    auto config = make_config(GaitPattern::Tripod, 2.5);
    config.step_length_m = 0.04;

    const double v_max = max_forward_speed_mps(config);
    EXPECT_NEAR(v_max, 0.2, 1e-12);  // 0.04 / (0.5 / 2.5) = 0.2 m/s

    // 一致性：v_max × 支撑相时长 == 步长上限
    EXPECT_NEAR(v_max * stance_duration_s(config), config.step_length_m, 1e-12);
}

TEST(GaitTypesTest, MaxLateralSpeedAndTurnRateUseTheirOwnLimits) {
    auto config = make_config(GaitPattern::Tripod, 2.5);
    config.lateral_step_m = 0.02;
    config.turn_step_rad = 0.15;

    EXPECT_NEAR(max_lateral_speed_mps(config), 0.02 / 0.2, 1e-12);  // 0.1 m/s
    EXPECT_NEAR(max_turn_rate_rps(config), 0.15 / 0.2, 1e-12);      // 0.75 rad/s

    EXPECT_NEAR(
        max_lateral_speed_mps(config) * stance_duration_s(config), config.lateral_step_m, 1e-12);
    EXPECT_NEAR(max_turn_rate_rps(config) * stance_duration_s(config), config.turn_step_rad, 1e-12);
}

// 默认参数必须自洽：默认配置能支持 0.2 m/s，也就是 GaitConfig 里那组默认值
// 和"给 leg_calc 发 vx=0.2"这个常用测试命令是匹配的。
TEST(GaitTypesTest, DefaultConfigSupportsTheUsualTestCommand) {
    const GaitConfig config;  // 全默认
    // 0.04 和 0.2 都不是二进制精确值，理论值 0.04 / 0.2 算出来可能是 0.19999999999999998，
    // 所以直接和 0.2 比较会失败。这里留一点容差。
    EXPECT_GE(max_forward_speed_mps(config), 0.2 - 1e-9)
        << "默认参数应能支持 vx = 0.2 m/s，否则常用测试命令会被静默夹取";
}

TEST(GaitTypesTest, RaisingFrequencyRaisesMaxSpeed) {
    auto slow = make_config(GaitPattern::Tripod, 1.0);
    auto fast = make_config(GaitPattern::Tripod, 2.0);
    slow.step_length_m = fast.step_length_m = 0.04;

    EXPECT_NEAR(max_forward_speed_mps(fast), 2.0 * max_forward_speed_mps(slow), 1e-12);
}

// ---------------------------------------------------------------------------
// quintic_ease：起步 / 停步过渡曲线
//
//   s(τ) = 10τ³ − 15τ⁴ + 6τ⁵
//
// 6 个边界条件（两端的位置 / 速度 / 加速度）全部由这 6 个系数满足。
// 这里逐条验，因为任何一条被破坏都会让过渡产生冲击。
// ---------------------------------------------------------------------------
TEST(QuinticEaseTest, EndpointsAreZeroAndOne) {
    EXPECT_DOUBLE_EQ(quintic_ease(0.0), 0.0);
    EXPECT_DOUBLE_EQ(quintic_ease(1.0), 1.0);
    EXPECT_DOUBLE_EQ(quintic_ease(0.5), 0.5);
}

TEST(QuinticEaseTest, EndpointsHaveZeroVelocity) {
    constexpr double h = 1e-4;
    EXPECT_NEAR(one_sided_derivative_at_zero(h), 0.0, 1e-6)
        << "起点速度必须为 0，否则过渡开始的瞬间就有速度跳变";
    EXPECT_NEAR(one_sided_derivative_at_one(h), 0.0, 1e-6)
        << "终点速度必须为 0，否则过渡结束的瞬间就有速度跳变";
}

TEST(QuinticEaseTest, AccelerationVanishesAtTheEndpoints) {
    // s''(τ) = 60τ − 180τ² + 120τ³ 在 τ = 0 处恰好为 0，但它随 τ **线性增长**，
    // 所以在 τ = h 处已经能测到约 60h 的加速度。
    // 因此"端点加速度为 0"不能用固定步长的差分直接断言等于 0，
    // 正确的数值表述是：**步长越小，差分值也越小**（三次多项式的加速度在这里是常数，
    // 不会随 h 减小）。
    double previous = 1e9;
    for (const double h : {1e-2, 1e-3, 1e-4}) {
        const double accel =
            (quintic_ease(2.0 * h) - 2.0 * quintic_ease(h) + quintic_ease(0.0)) / (h * h);
        EXPECT_LT(accel, previous) << "步长减小时差分值必须跟着减小（说明 a(0)=0），h=" << h;
        EXPECT_LT(accel, 100.0 * h) << "差分值应与解析量级 60h 一致，h=" << h;
        previous = accel;
    }
}

TEST(QuinticEaseTest, IsMonotonicAndSymmetric) {
    double previous = quintic_ease(0.0);
    for (int i = 1; i <= 100; ++i) {
        const double tau = static_cast<double>(i) / 100.0;
        const double value = quintic_ease(tau);
        EXPECT_GE(value, previous - 1e-15) << "曲线必须单调不减，tau=" << tau;
        previous = value;
    }

    // 中心对称：s(τ) + s(1-τ) = 1
    for (int i = 0; i <= 20; ++i) {
        const double tau = static_cast<double>(i) / 20.0;
        EXPECT_NEAR(quintic_ease(tau) + quintic_ease(1.0 - tau), 1.0, 1e-12)
            << "tau=" << tau;
    }
}

TEST(QuinticEaseTest, ClampsOutsideUnitRange) {
    EXPECT_DOUBLE_EQ(quintic_ease(-1.0), 0.0);
    EXPECT_DOUBLE_EQ(quintic_ease(-1e-9), 0.0);
    EXPECT_DOUBLE_EQ(quintic_ease(2.0), 1.0);
    EXPECT_DOUBLE_EQ(quintic_ease(1.0 + 1e-9), 1.0);
}
