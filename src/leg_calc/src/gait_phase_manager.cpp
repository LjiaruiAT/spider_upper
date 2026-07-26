#include "leg_calc/gait_phase_manager.hpp"

#include <algorithm>
#include <cmath>

namespace leg_calc {

GaitPhaseManager::GaitPhaseManager(const GaitConfig& config)
    : config_(config), elapsed_(0.0) {
    // 初始化：三足 A 在支撑相，三足 B 在摆动相
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (is_tripod_a(i)) {
            state_.phases[i] = LegPhase::Stance;
        } else {
            state_.phases[i] = LegPhase::Swing;
            state_.phase_fraction[i] = 0.0;
        }
    }
}

void GaitPhaseManager::tick(double dt_s) {
    elapsed_ += dt_s;

    const double period_s = 1.0 / std::max(config_.frequency_hz, 0.01);

    switch (config_.pattern) {
    case GaitPattern::Tripod:
        update_tripod(period_s);
        break;
    case GaitPattern::Wave:
        update_wave(period_s);
        break;
    case GaitPattern::Ripple:
        update_ripple(period_s);
        break;
    }
}

void GaitPhaseManager::reset() {
    elapsed_ = 0.0;
    state_ = GaitState{};
    for (std::size_t i = 0; i < kLegCount; ++i) {
        if (is_tripod_a(i)) {
            state_.phases[i] = LegPhase::Stance;
        } else {
            state_.phases[i] = LegPhase::Swing;
        }
    }
}

void GaitPhaseManager::update_config(const GaitConfig& config) {
    config_ = config;
    // 配置变更时不重置时钟，只调整后续行为
}

void GaitPhaseManager::update_tripod(double period_s) {
    // 全局相位 [0, 1)，一个完整步态周期
    state_.global_phase = std::fmod(elapsed_ / period_s, 1.0);

    // 三足步态：两个半周期
    // 前半周期 (0 ~ 0.5): Tripod A = Stance, Tripod B = Swing
    // 后半周期 (0.5 ~ 1.0): Tripod A = Swing, Tripod B = Stance
    const bool first_half = state_.global_phase < 0.5;

    for (std::size_t i = 0; i < kLegCount; ++i) {
        const bool is_a = is_tripod_a(i);

        if (first_half) {
            // A 支撑, B 摆动
            state_.phases[i] = is_a ? LegPhase::Stance : LegPhase::Swing;
        } else {
            // A 摆动, B 支撑
            state_.phases[i] = is_a ? LegPhase::Swing : LegPhase::Stance;
        }

        // 计算在当前相位内的进度 [0, 1)
        if (first_half) {
            state_.phase_fraction[i] = state_.global_phase * 2.0;  // 拉伸到 [0, 1)
        } else {
            state_.phase_fraction[i] = (state_.global_phase - 0.5) * 2.0;
        }
    }
}

void GaitPhaseManager::update_wave(double period_s) {
    // 波动步态：每次只有一条腿在摆动
    // 六条腿按顺序依次摆动：LF → LM → LR → RR → RM → RF
    state_.global_phase = std::fmod(elapsed_ / period_s, 1.0);

    for (std::size_t i = 0; i < kLegCount; ++i) {
        // 每条腿的摆动窗口占 1/6 周期，偏移量不同
        const double leg_offset = static_cast<double>(i) / static_cast<double>(kLegCount);
        double leg_phase = std::fmod(state_.global_phase + 1.0 - leg_offset, 1.0);

        if (leg_phase < 1.0 / static_cast<double>(kLegCount)) {
            state_.phases[i] = LegPhase::Swing;
            state_.phase_fraction[i] = leg_phase * static_cast<double>(kLegCount);
        } else {
            state_.phases[i] = LegPhase::Stance;
            state_.phase_fraction[i] = (leg_phase - 1.0 / kLegCount) /
                                        (1.0 - 1.0 / kLegCount);
        }
    }
}

void GaitPhaseManager::update_ripple(double period_s) {
    // 波纹步态：每次两条腿摆动
    // 三组依次摆动：(LF+RR) → (LM+RM) → (LR+RF)
    state_.global_phase = std::fmod(elapsed_ / period_s, 1.0);

    constexpr std::size_t kGroupCount = 3;
    // Group 0: LF(0) + RR(5)
    // Group 1: LM(1) + RM(4)
    // Group 2: LR(2) + RF(3)
    constexpr std::size_t group_map[6] = {0, 1, 2, 2, 1, 0};

    for (std::size_t i = 0; i < kLegCount; ++i) {
        const double group_offset = static_cast<double>(group_map[i]) / static_cast<double>(kGroupCount);
        double leg_phase = std::fmod(state_.global_phase + 1.0 - group_offset, 1.0);

        if (leg_phase < 1.0 / static_cast<double>(kGroupCount)) {
            state_.phases[i] = LegPhase::Swing;
            state_.phase_fraction[i] = leg_phase * kGroupCount;
        } else {
            state_.phases[i] = LegPhase::Stance;
            state_.phase_fraction[i] = (leg_phase - 1.0 / kGroupCount) /
                                        (1.0 - 1.0 / kGroupCount);
        }
    }
}

}  // namespace leg_calc
