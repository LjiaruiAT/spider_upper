#pragma once

#include "leg_calc/gait_types.hpp"

namespace leg_calc {

// 步态相位管理器
// 根据时间和步态配置，计算六条腿当前的支撑/摆动相位
class GaitPhaseManager {
public:
    explicit GaitPhaseManager(const GaitConfig& config);

    // 每个控制周期调用一次，推进步态时间
    void tick(double dt_s);

    // 获取当前步态瞬时状态（只读）
    const GaitState& state() const { return state_; }

    // 重置步态时钟
    void reset();

    // 运行时更新配置（切换步态模式/频率等）
    void update_config(const GaitConfig& config);

private:
    void update_tripod(double period_s);
    void update_wave(double period_s);
    void update_ripple(double period_s);

    GaitConfig config_;
    GaitState state_;
    double elapsed_{0.0};
};

}  // namespace leg_calc
