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

// 步态配置参数。速度命令不直接存放在这里；这些字段决定轨迹的尺度、周期和上限。
//
// 注意：step_length_m / lateral_step_m / turn_step_rad 是**上限**，不是实际值。
// 实际步长由"速度命令 × 支撑相时长"决定（见 FootTrajectory::compute_step_command），
// 只有在命令速度超出机械能力时，才被这几个上限夹住。
struct GaitConfig {
    GaitPattern pattern{GaitPattern::Tripod};
    double frequency_hz{1.0};         // 步态频率，单位 Hz（每秒多少个完整步态周期）
    double step_length_m{0.04};       // 前进方向步长上限
    double step_height_m{0.03};       // 抬腿最大高度
    double lateral_step_m{0.02};      // 左右平移步长上限
    double turn_step_rad{0.15};       // 转向角度上限
    double body_height_m{0.12};       // 身体站立高度
};

// 支撑相占整个步态周期的比例。
//
// 为什么需要它：`速度 × 时间 = 位移`，而"时间"指的是支撑相真实持续了多久。
// 不同步态的时间结构不同，所以这个比例不能统一写死 0.5，
// 否则 Wave 步态下算出的步长会偏大 40% 以上。
inline constexpr double stance_duty(GaitPattern pattern) {
    switch (pattern) {
    case GaitPattern::Tripod:
        return 0.5;  // 两个三足组各占半个周期：一组支撑时另一组摆动
    case GaitPattern::Ripple:
        return 2.0 / 3.0;  // 三组轮流摆动，每组摆动窗口占 1/3
    case GaitPattern::Wave:
        return 5.0 / 6.0;  // 每次只有一条腿摆动，摆动窗口占 1/6
    }
    return 0.5;
}

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
