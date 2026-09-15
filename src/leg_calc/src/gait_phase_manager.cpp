#include "leg_calc/gait_phase_manager.hpp"
#include "leg_calc/common_types.hpp"

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
    // elapsed_ 是步态时钟；每次控制周期推进一次。
    elapsed_ += dt_s;

    // frequency_hz 表示一个完整全局周期每秒重复多少次。
    // 最小值保护避免频率为 0 时除零；period_s 单位为秒。
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
    // 把绝对时间折叠成一个周期内的无量纲相位：
    //   global_phase = (elapsed / period) mod 1，范围 [0, 1)。
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

        // 当前相位只占半个全局周期，所以要乘 2 拉伸回 [0, 1)。
        // 这个 fraction 会直接作为 FootTrajectory 的插值参数。
        if (first_half) {
            state_.phase_fraction[i] = state_.global_phase * 2.0;
        } else {
            state_.phase_fraction[i] = (state_.global_phase - 0.5) * 2.0;
        }
    }
}

void GaitPhaseManager::update_wave(double period_s) {
    // 波动步态：每次只有一条腿在摆动。
    // 每条腿拥有一个长度为 1/6 的摆动窗口，其余时间为支撑相。
    // 注意：这里的实际顺序由 LegId 数值索引决定，当前实现是 LF, LM, LR, RF, RM, RR；
    // 注释中的工程期望顺序若要改变，需要同时修改 offset 计算或腿索引映射。
    state_.global_phase = std::fmod(elapsed_ / period_s, 1.0);

    for (std::size_t i = 0; i < kLegCount; ++i) {
        // leg_phase = (global_phase - leg_offset) mod 1，把不同腿的时间偏移折叠到 [0,1)。
        // 摆动窗口长度是 1/6；窗口外的时间按比例映射成支撑相进度。
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
    // 波纹步态：每次两条腿摆动；三组各占全局周期的 1/3。
    // 组内腿共享同一个 leg_phase，因此会同步进入/离开摆动相。
    state_.global_phase = std::fmod(elapsed_ / period_s, 1.0);

    constexpr std::size_t kGroupCount = 3;
    // Group 0: LF(0) + RR(5)
    // Group 1: LM(1) + RM(4)
    // Group 2: LR(2) + RF(3)
    constexpr std::size_t group_map[6] = {0, 1, 2, 2, 1, 0};

    for (std::size_t i = 0; i < kLegCount; ++i) {
        // 对波纹步态也是同样的“偏移 -> 折叠 -> 判断摆动窗口”过程，只是三组共享偏移。
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
