#pragma once

#include "leg_calc/common_types.hpp"
#include "leg_calc/gait_types.hpp"

#include <Eigen/Dense>

namespace leg_calc {

// 足端轨迹生成器。
// 它是 IK 之前的一层：根据“当前是哪一相”和“身体想怎么动”，生成目标足端位置。
// 输入和输出都以身体坐标系为主，之后由 leg_calc 转到单腿局部坐标系做 IK。
// 注意：轨迹层只负责给出目标 p，不负责判断 p 是否可达，也不负责求关节角。
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

    // 运行时更新步态配置（步长上限、高度等）
    void update_config(const GaitConfig& config) {
        config_ = config;
    }

private:
    // 一个支撑相内，足端相对身体需要移动的"全步长"和"全转角"。
    // translation：全步长，单位 m；turn_rad：全转角，单位 rad。
    struct StepCommand {
        Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
        double turn_rad{0.0};
    };

    // 把"速度 / 角速度命令"换算成"一个支撑相内的位移 / 转角"。
    // 这是整个轨迹层唯一一处"速度 -> 位移"的换算，支撑相与摆动相共用。
    StepCommand compute_step_command(const BodyTwist& body_twist) const;

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
};

}  // namespace leg_calc
