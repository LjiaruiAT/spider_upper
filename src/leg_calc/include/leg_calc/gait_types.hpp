#pragma once

#include <cstddef>

namespace leg_calc {

// 步态模式。它只决定每条腿处于支撑相还是摆动相；真正的足端位置
// 由 FootTrajectory 根据相位和速度命令继续计算，最后才进入 IK。
enum class GaitPattern {
    Tripod,  // 三足步态：LF+LR+RM vs RF+RR+LM，交替支撑
    Ripple,  // 波纹步态：每次移动 2 条腿（预留）
    Wave,    // 波动步态：每次移动 1 条腿（预留）
};

// 步态配置参数。速度命令不直接存放在这里；这些字段主要决定轨迹的尺度和周期。
struct GaitConfig {
    GaitPattern pattern{GaitPattern::Tripod};
    double frequency_hz{1.0};         // 步态频率
    double step_length_m{0.04};       // 前进方向的步长
    double step_height_m{0.03};       // 抬腿最大高度
    double lateral_step_m{0.02};      // 左右平移步长
    double turn_step_rad{0.15};       // 转向角度步长
    double body_height_m{0.12};       // 身体站立高度
};

// 单腿相位状态。Stance 时脚应近似留在地面，Swing 时脚离地移动到下一个落脚点。
enum class LegPhase {
    Stance,  // 支撑相：脚在地上，向后推
    Swing,   // 摆动相：脚在空中，向前移
};

// 六足步态瞬时状态。
// phase_fraction 不直接表示全局时间，而是表示“这条腿在当前 Stance/Swing 相位内走了多少”，范围 [0, 1)。
struct GaitState {
    LegPhase phases[6]{};         // 每条腿的当前相位
    double phase_fraction[6]{};   // 每条腿在当前相位内的进度 [0, 1)
    double global_phase{0.0};     // 全局步态周期相位 [0, 1)

    GaitState() {
        for (std::size_t i = 0; i < 6; ++i) {
            phases[i] = LegPhase::Stance;
            phase_fraction[i] = 0.0;
        }
    }
};

// 三足步态分组
// Tripod A: LeftFront, LeftRear, RightMiddle
// Tripod B: RightFront, RightRear, LeftMiddle
inline constexpr bool is_tripod_a(std::size_t leg_index) {
    // LegId order: LF=0, LM=1, LR=2, RF=3, RM=4, RR=5。
    // 这个索引顺序必须和 SpiderFootTargets、SpiderJointTargets、Servo18 映射一致。
    return leg_index == 0 || leg_index == 2 || leg_index == 4;
}

inline constexpr bool is_tripod_b(std::size_t leg_index) {
    return leg_index == 1 || leg_index == 3 || leg_index == 5;
}

}  // namespace leg_calc
