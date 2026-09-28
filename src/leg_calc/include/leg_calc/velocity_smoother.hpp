#pragma once

#include "leg_calc/common_types.hpp"
#include "leg_calc/gait_types.hpp"

namespace leg_calc {

// 速度命令平滑器（slew rate limiter）。
//
// 为什么需要它：
//
//   `足端位移 = 速度 × 支撑相时长` 这个式子是**瞬时成立**的——速度命令一变，
//   步长在同一个控制周期内就跟着变。命令从 0.2 变成 −0.2 时，步长从 +40mm
//   跳到 −40mm，足端一步跨出 80mm。真机上这就是舵机被要求瞬间大幅转动。
//
//   起步 / 停步那条路径已经由 motion_scale 处理（它还能缩放抬腿高度），
//   但"运行中修改速度"没有对应机制，本类补的正是这一块。
//
// 职责边界：
//   · 做   —— 把当前速度按固定的最大变化率向目标逼近
//   · 不做 —— 启停判断、抬腿高度缩放、命令是否超能力上限的判断
//
// 状态性：本类持有"当前速度"，所以节点必须长期持有同一个实例。
// 完全停下后调用 reset()，让下次起步总从零开始。
class VelocitySmoother {
public:
    explicit VelocitySmoother(const GaitConfig& config);

    // 运行时更新参数（加速度上限）
    void update_config(const GaitConfig& config) { config_ = config; }

    // 把当前速度向 target 逼近一步，单步变化量不超过「加速度上限 × dt」。
    // 返回逼近后的速度；不修改传入的 target。dt <= 0 时不推进。
    BodyTwist update(const BodyTwist& target, double dt);

    // 回到零速度。
    void reset() { current_ = BodyTwist{}; }

    const BodyTwist& current() const { return current_; }

private:
    // 单分量逼近：一步跨不过去就按 max_delta 走，够得着就直接落位。
    // 后一条是必须的——否则会在目标附近以 max_delta 为幅度来回抖动。
    static double approach(double current, double target, double max_delta);

    GaitConfig config_;
    BodyTwist current_{};
};

}  // namespace leg_calc
