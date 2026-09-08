#pragma once

#include "leg_calc/common_types.hpp"
#include "leg_calc/gait_types.hpp"

#include <Eigen/Dense>

namespace leg_calc {

// 足端轨迹生成器
// 根据步态相位和身体速度，计算每条腿的足端目标位置
// 输入和输出都以身体坐标系为主，后续再由 leg_calc 转到单腿局部坐标系做 IK。
class FootTrajectory {
public:
    explicit FootTrajectory(const GaitConfig& config);

    // 计算单腿在当前步态状态下的足端目标（身体坐标系）
    Eigen::Vector3d compute_foot_target(
        LegId leg_id,
        const Eigen::Vector3d& nominal_foot,
        LegPhase phase,
        double phase_fraction,
        const BodyTwist& body_twist);

    // 运行时更新步态配置（步长、高度等）
    void update_config(const GaitConfig& config) {
        config_ = config;
        step_half_ = config_.step_length_m * 0.5;
    }

private:
    // 支撑相轨迹：足端向后滑动，推动身体前进
    Eigen::Vector3d stance_trajectory(
        LegId leg_id,
        const Eigen::Vector3d& nominal,
        double fraction,
        const BodyTwist& body_twist);

    // 摆动相轨迹：摆线弧线，抬腿→前移→落地
    Eigen::Vector3d swing_trajectory(
        LegId leg_id,
        const Eigen::Vector3d& nominal,
        double fraction,
        const BodyTwist& body_twist);

    GaitConfig config_;
    double step_half_{0.02};  // step_length_m / 2，缓存用
};

}  // namespace leg_calc
