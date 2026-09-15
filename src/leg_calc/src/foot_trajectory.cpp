#include "leg_calc/foot_trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace leg_calc {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d apply_planar_turn(const Eigen::Vector3d& nominal, double turn) {
    // 小角度平面旋转的一阶近似：
    //   x' ≈ x - y*turn, y' ≈ y + x*turn。
    // 当前没有使用完整 sin/cos 旋转矩阵，因此 turn 应理解为小角度近似量。
    Eigen::Vector3d rotated = nominal;
    rotated.x() += -nominal.y() * turn;
    rotated.y() += nominal.x() * turn;
    return rotated;
}

}  // namespace

FootTrajectory::FootTrajectory(const GaitConfig& config)
    : config_(config), step_half_(config.step_length_m * 0.5) {}

Eigen::Vector3d FootTrajectory::compute_foot_target(
    LegId leg_id,
    const Eigen::Vector3d& nominal_foot,
    LegPhase phase,
    double phase_fraction,
    const BodyTwist& body_twist) {
    // 相位只决定使用哪条轨迹；两条轨迹的返回值都是身体坐标系下的足端目标，
    // 还要经过 body_point_to_leg_point 才能作为 LegKinematics::inverse_position 的输入。
    if (phase == LegPhase::Stance) {
        return stance_trajectory(leg_id, nominal_foot, phase_fraction, body_twist);
    }
    return swing_trajectory(leg_id, nominal_foot, phase_fraction, body_twist);
}

Eigen::Vector3d FootTrajectory::stance_trajectory(
    LegId leg_id,
    const Eigen::Vector3d& nominal,
    double fraction,
    const BodyTwist& body_twist) {
    (void)leg_id;

    // 支撑相假设脚掌仍与地面接触：身体向前走时，足端相对身体向后移动。
    // centered 在 fraction=0 时为 +1，在 fraction=0.5 时为 0，在 fraction=1 时为 -1，
    // 所以轨迹以 nominal 为中心，前后各偏移 step_half_。
    const double centered = 1.0 - (2.0 * fraction);
    const Eigen::Vector3d planar_motion(
        body_twist.linear.x() * step_half_ * centered,
        body_twist.linear.y() * step_half_ * centered,
        0.0);

    const double turn = body_twist.angular.z() * config_.turn_step_rad * centered;
    // 线速度产生平移目标，角速度产生绕身体 z 轴的平面转动目标。
    // 这里的 turn 是启发式小角度量，不是完整的刚体积分。
    Eigen::Vector3d target = nominal - planar_motion;
    target = apply_planar_turn(target, -turn);
    return target;
}

Eigen::Vector3d FootTrajectory::swing_trajectory(
    LegId leg_id,
    const Eigen::Vector3d& nominal,
    double fraction,
    const BodyTwist& body_twist) {
    (void)leg_id;

    // 摆动相分为：从 nominal 后方开始 -> 向前摆 -> 回到 nominal 前方。
    // progress 范围 [-1, +1]，这样平移位移与支撑相在 nominal 两侧衔接。
    const double progress = 2.0 * fraction - 1.0;
    // sin(pi*fraction) 在相位起止点为 0、中点为 1，形成平滑的抬脚高度。
    const double lift = std::sin(kPi * std::clamp(fraction, 0.0, 1.0));
    const Eigen::Vector3d planar_motion(
        body_twist.linear.x() * step_half_ * progress,
        body_twist.linear.y() * step_half_ * progress,
        0.0);

    const double turn = body_twist.angular.z() * config_.turn_step_rad * progress;
    Eigen::Vector3d target = nominal + planar_motion;
    target = apply_planar_turn(target, turn);
    target.z() += config_.step_height_m * lift;
    return target;
}

}  // namespace leg_calc
