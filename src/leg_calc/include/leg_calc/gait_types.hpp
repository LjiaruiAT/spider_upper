#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

namespace leg_calc {

// 步态模式。它只决定每条腿处于支撑相还是摆动相；真正的足端位置
// 由 FootTrajectory 根据相位和速度命令继续计算，最后才进入 IK。
//
// 三种步态都可用，启动时通过 ROS 参数 `gait_pattern` 选择（见节点与 5.18 节）。
// 它们的区别只在"同时有几条腿摆动"，这决定了支撑相占比、进而决定速度上限：
//   支撑相占比  Tripod 1/2 < Ripple 2/3 < Wave 5/6
//   速度上限    Tripod 最高       Ripple 中       Wave 最低
enum class GaitPattern {
    Tripod,  // 三足步态：LF+LR+RM vs RF+RR+LM，交替支撑（最快，默认）
    Ripple,  // 波纹步态：每次移动 2 条腿（稳定性与速度的折中）
    Wave,    // 波动步态：每次移动 1 条腿（最稳，但速度上限最低）
};

// 步态名称 <-> 枚举的互转。
// 节点的 gait_pattern 参数与启动打印都走这两个函数，
// 保证"配置里写的字符串"和"日志里打的名字"来自同一处定义，不会对不上。
inline const char* gait_pattern_name(GaitPattern pattern) {
    switch (pattern) {
    case GaitPattern::Tripod:
        return "tripod";
    case GaitPattern::Ripple:
        return "ripple";
    case GaitPattern::Wave:
        return "wave";
    }
    return "unknown";
}

// 解析失败抛 std::invalid_argument——与 leg_layout 对配置错误的处理一致：
// 宁可启动失败，也不要静默退回默认步态（那会让人以为在跑 Wave，实际在跑 Tripod）。
inline GaitPattern parse_gait_pattern(const std::string& name) {
    if (name == "tripod") {
        return GaitPattern::Tripod;
    }
    if (name == "ripple") {
        return GaitPattern::Ripple;
    }
    if (name == "wave") {
        return GaitPattern::Wave;
    }
    throw std::invalid_argument(
        "Unknown gait pattern: '" + name + "' (expected tripod / ripple / wave)");
}

// 步态配置参数。速度命令不直接存放在这里；这些字段决定轨迹的尺度、周期和上限。
//
// 注意：step_length_m / lateral_step_m / turn_step_rad 是**上限**，不是实际值。
// 实际步长由"速度命令 × 支撑相时长"决定（见 FootTrajectory::compute_step_command），
// 只有在命令速度超出机械能力时，才被这几个上限夹住。
//
// 这几个参数是**相互耦合**的：步长上限 + 步态频率 + 支撑相占比 三者一确定，
// 能支持的最大速度也就确定了（见 max_forward_speed_mps）。改一个必须回头看另外两个，
// 否则就会出现"命令 0.2 m/s、实际只走 0.08 m/s"这种命令与现实脱节的情况。
//
// 关于"身体站立高度"：它**不在**这里。站姿由 spider/config/leg_params.yaml 提供
// （`body_height_mm`），因为那是机器人的几何属性，不是步态参数。
struct GaitConfig {
    GaitPattern pattern{GaitPattern::Tripod};

    // 步态频率。提高它能提高最大速度，但会压缩摆动相时间；
    // 摆动相要在 (1 - stance_duty) / frequency 秒内完成"抬腿 + 前摆 + 落地"，
    // 所以上限最终由舵机角速度决定。
    double frequency_hz{2.5};

    double step_length_m{0.04};   // 前进方向步长上限
    double step_height_m{0.03};   // 抬腿最大高度
    double lateral_step_m{0.02};  // 左右平移步长上限
    double turn_step_rad{0.15};   // 转向角度上限

    // 速度命令的最大变化率，也就是"加速度上限"。用来抑制**运行中命令突变**。
    //
    // 轨迹层的位移 = 速度 × 支撑相时长，这个式子是瞬时成立的：速度命令一变，
    // 步长在同一个控制周期内就跟着变。命令从 0.2 变成 −0.2 时步长从 +40mm
    // 跳到 −40mm，足端一步跨出 80mm——真机上就是舵机被要求瞬间大幅转动。
    //
    // 它只管"运行中变速"，不管启停：启停由 motion_scale 负责（那个还顺带缩放
    // 抬腿高度，光靠加速度上限做不到）。所以节点只在"想运动"时推进平滑器。
    //
    // 0 或负值 = 不做平滑，直接跟随命令（旧行为）。
    // 默认 0.4 使得 0 → max_forward_speed(0.2 m/s) 需要 0.5 s。
    double max_linear_accel_mps2{0.4};
    double max_angular_accel_rps2{1.0};
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

// 一个支撑相真实持续的时间（秒）= 支撑相占比 ÷ 步态频率。
// 这是"速度 × 时间 = 位移"里那个"时间"，轨迹层和上面的能力上限共用它，
// 避免两处各算一遍导致口径不一致。
inline double stance_duration_s(const GaitConfig& config) {
    constexpr double kMinFrequencyHz = 0.01;  // 防止除零
    const double frequency_hz = config.frequency_hz > kMinFrequencyHz ? config.frequency_hz : kMinFrequencyHz;
    return stance_duty(config.pattern) / frequency_hz;
}

// 当前参数能支持的最大前进速度（m/s），即"再快就会被步长上限夹住"的那个点：
//   v_max = step_length_m ÷ 支撑相时长
// 命令超过它时机器人**不会走得更快**，只是命令与现实脱节。
// 节点启动时会打印它，命令超限时也会报警。
inline double max_forward_speed_mps(const GaitConfig& config) {
    return config.step_length_m / stance_duration_s(config);
}

inline double max_lateral_speed_mps(const GaitConfig& config) {
    return config.lateral_step_m / stance_duration_s(config);
}

inline double max_turn_rate_rps(const GaitConfig& config) {
    return config.turn_step_rad / stance_duration_s(config);
}

// 五次多项式缓动（quintic smoothstep）：把 [0, 1] 映射到 [0, 1]。
//
//   s(τ) = 10τ³ − 15τ⁴ + 6τ⁵
//
// 这个多项式的 6 个系数由 6 个边界条件唯一确定：
//   s(0)=0, s(1)=1        （位置）
//   s'(0)=0, s'(1)=0      （速度）
//   s''(0)=0, s''(1)=0    （加速度）
//
// 用三次多项式只能满足前 4 个条件，加速度在两端不连续——那意味着冲击。
// 五次把加速度也约束住，所以过渡过程中位置、速度、加速度都是连续的。
//
// 用途：起步/停步时的"运动强度"过渡（见 LegCalcNode 的 motion_scale）。
inline constexpr double quintic_ease(double tau) {
    const double t = tau < 0.0 ? 0.0 : (tau > 1.0 ? 1.0 : tau);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
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
