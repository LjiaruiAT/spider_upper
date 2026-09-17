// GaitPhaseManager 单元测试
//
// 相位管理器的职责只有一个：根据离散时间推进，算出六条腿此刻处于
// 支撑相还是摆动相。它不产生足端坐标（那是 FootTrajectory 的事）。
//
// 这里锁定三件事：
//   1. 不同步态结构下"同时处于摆动相的腿数"必须正确
//      （Tripod=3、Wave=1、Ripple=2）
//   2. Tripod 的分组必须是 LF/LR/RM 对 LM/RF/RR，且两半周期互换
//   3. phase_fraction / global_phase 始终落在 [0, 1)
//
// 运行：colcon test --packages-select leg_calc

#include <gtest/gtest.h>

#include <cstddef>

// LegId / kLegCount / leg_index 定义在 common_types.hpp；
// gait_phase_manager.hpp 自身只依赖 gait_types.hpp，所以测试要显式包含。
#include "leg_calc/common_types.hpp"
#include "leg_calc/gait_phase_manager.hpp"

namespace {

using leg_calc::GaitConfig;
using leg_calc::GaitPattern;
using leg_calc::GaitPhaseManager;
using leg_calc::LegId;
using leg_calc::LegPhase;

constexpr std::size_t kLegCount = leg_calc::kLegCount;

// 控制周期取 10ms、步态频率 1Hz，这样一个完整周期正好 100 次 tick，
// 每个采样点都能落在一个干净的相位上。
constexpr double kDt = 0.01;
constexpr int kTicksPerPeriod = 100;

GaitConfig make_config(GaitPattern pattern) {
    GaitConfig config;
    config.pattern = pattern;
    config.frequency_hz = 1.0;
    return config;
}

std::size_t count_phase(const leg_calc::GaitState& state, LegPhase phase) {
    std::size_t count = 0;
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        if (state.phases[leg] == phase) {
            ++count;
        }
    }
    return count;
}

bool phase_of(const leg_calc::GaitState& state, LegId leg) {
    return state.phases[leg_calc::leg_index(leg)] == LegPhase::Stance;
}

// 推进若干个控制周期
void advance(GaitPhaseManager& manager, int ticks) {
    for (int i = 0; i < ticks; ++i) {
        manager.tick(kDt);
    }
}

}  // namespace

// ---------------------------------------------------------------------------
// 同时处于摆动相的腿数
//
// 这是每种步态最本质的特征：它决定了"任何时刻有几条腿在支撑身体"。
// 数量错了，机器人要么站不住（支撑腿太少），要么步态不像设计的那样。
// ---------------------------------------------------------------------------
TEST(GaitPhaseManagerTest, TripodAlwaysHasThreeLegsInStance) {
    GaitPhaseManager manager(make_config(GaitPattern::Tripod));

    // 走满两个完整周期，逐拍检查
    for (int tick = 0; tick < 2 * kTicksPerPeriod; ++tick) {
        manager.tick(kDt);
        ASSERT_EQ(count_phase(manager.state(), LegPhase::Stance), 3u)
            << "第 " << tick << " 拍：三足步态必须恰好 3 条腿支撑";
        ASSERT_EQ(count_phase(manager.state(), LegPhase::Swing), 3u);
    }
}

TEST(GaitPhaseManagerTest, WaveAlwaysHasExactlyOneLegInSwing) {
    GaitPhaseManager manager(make_config(GaitPattern::Wave));

    for (int tick = 0; tick < 2 * kTicksPerPeriod; ++tick) {
        manager.tick(kDt);
        ASSERT_EQ(count_phase(manager.state(), LegPhase::Swing), 1u)
            << "第 " << tick << " 拍：波动步态每次只能有 1 条腿摆动";
    }
}

TEST(GaitPhaseManagerTest, RippleAlwaysHasExactlyTwoLegsInSwing) {
    GaitPhaseManager manager(make_config(GaitPattern::Ripple));

    for (int tick = 0; tick < 2 * kTicksPerPeriod; ++tick) {
        manager.tick(kDt);
        ASSERT_EQ(count_phase(manager.state(), LegPhase::Swing), 2u)
            << "第 " << tick << " 拍：波纹步态每次应有 2 条腿摆动";
    }
}

// ---------------------------------------------------------------------------
// Tripod 分组
//
// A 组：LF(0) + LR(2) + RM(4)
// B 组：LM(1) + RF(3) + RR(5)
// 两半周期必须互换，否则六条腿的相位关系就乱了。
// ---------------------------------------------------------------------------
TEST(GaitPhaseManagerTest, TripodGroupsSwapEveryHalfPeriod) {
    GaitPhaseManager manager(make_config(GaitPattern::Tripod));

    // 推进到周期的 1/4 处（global_phase ≈ 0.25，属于前半周期）
    advance(manager, kTicksPerPeriod / 4);

    EXPECT_TRUE(phase_of(manager.state(), LegId::LeftFront));
    EXPECT_TRUE(phase_of(manager.state(), LegId::LeftRear));
    EXPECT_TRUE(phase_of(manager.state(), LegId::RightMiddle));
    EXPECT_FALSE(phase_of(manager.state(), LegId::LeftMiddle));
    EXPECT_FALSE(phase_of(manager.state(), LegId::RightFront));
    EXPECT_FALSE(phase_of(manager.state(), LegId::RightRear));

    // 再推进半个周期到 global_phase ≈ 0.75（后半周期），两组应完全互换
    advance(manager, kTicksPerPeriod / 2);

    EXPECT_FALSE(phase_of(manager.state(), LegId::LeftFront));
    EXPECT_FALSE(phase_of(manager.state(), LegId::LeftRear));
    EXPECT_FALSE(phase_of(manager.state(), LegId::RightMiddle));
    EXPECT_TRUE(phase_of(manager.state(), LegId::LeftMiddle));
    EXPECT_TRUE(phase_of(manager.state(), LegId::RightFront));
    EXPECT_TRUE(phase_of(manager.state(), LegId::RightRear));
}

// ---------------------------------------------------------------------------
// 相位时钟
// ---------------------------------------------------------------------------
TEST(GaitPhaseManagerTest, GlobalPhaseCompletesOneCyclePerPeriod) {
    GaitPhaseManager manager(make_config(GaitPattern::Tripod));

    advance(manager, kTicksPerPeriod / 2);
    EXPECT_NEAR(manager.state().global_phase, 0.5, 1e-9)
        << "半个周期后全局相位应为 0.5";

    advance(manager, kTicksPerPeriod / 2);
    EXPECT_NEAR(manager.state().global_phase, 0.0, 1e-9)
        << "一个完整周期后全局相位应回到 0";
}

TEST(GaitPhaseManagerTest, FrequenciesScaleTheCycleLength) {
    auto slow = make_config(GaitPattern::Tripod);       // 1 Hz -> 周期 1.0s
    auto fast = make_config(GaitPattern::Tripod);
    fast.frequency_hz = 2.0;                            // 2 Hz -> 周期 0.5s

    GaitPhaseManager slow_manager(slow);
    GaitPhaseManager fast_manager(fast);

    advance(slow_manager, kTicksPerPeriod / 2);   // 0.5s
    advance(fast_manager, kTicksPerPeriod / 2);   // 0.5s = 一个完整周期

    EXPECT_NEAR(slow_manager.state().global_phase, 0.5, 1e-9);
    EXPECT_NEAR(fast_manager.state().global_phase, 0.0, 1e-9);
}

// ---------------------------------------------------------------------------
// 取值范围
//
// phase_fraction 会被直接当作 FootTrajectory 的插值参数，
// 越界会导致足端目标跑到设计之外。
// ---------------------------------------------------------------------------
TEST(GaitPhaseManagerTest, FractionsStayInUnitRangeForAllPatterns) {
    for (const auto pattern : {GaitPattern::Tripod, GaitPattern::Wave, GaitPattern::Ripple}) {
        GaitPhaseManager manager(make_config(pattern));

        for (int tick = 0; tick < 3 * kTicksPerPeriod; ++tick) {
            manager.tick(kDt);

            const auto& state = manager.state();
            ASSERT_GE(state.global_phase, 0.0);
            ASSERT_LT(state.global_phase, 1.0) << "pattern=" << static_cast<int>(pattern);

            for (std::size_t leg = 0; leg < kLegCount; ++leg) {
                ASSERT_GE(state.phase_fraction[leg], 0.0);
                ASSERT_LT(state.phase_fraction[leg], 1.0)
                    << "pattern=" << static_cast<int>(pattern) << " leg=" << leg;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 重置
// ---------------------------------------------------------------------------
TEST(GaitPhaseManagerTest, ResetReturnsToInitialState) {
    GaitPhaseManager manager(make_config(GaitPattern::Tripod));

    const auto initial = manager.state();
    advance(manager, kTicksPerPeriod / 3);
    ASSERT_NE(manager.state().global_phase, initial.global_phase);

    manager.reset();

    EXPECT_NEAR(manager.state().global_phase, initial.global_phase, 1e-12);
    for (std::size_t leg = 0; leg < kLegCount; ++leg) {
        EXPECT_EQ(manager.state().phases[leg], initial.phases[leg]);
        EXPECT_NEAR(manager.state().phase_fraction[leg], initial.phase_fraction[leg], 1e-12);
    }
}
