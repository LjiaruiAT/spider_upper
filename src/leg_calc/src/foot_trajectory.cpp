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
    : config_(config) {}

FootTrajectory::StepCommand FootTrajectory::compute_step_command(const BodyTwist& body_twist) const {
    // 支撑相时长 = 支撑相占周期的比例 ÷ 步态频率。
    // frequency_hz 表示"每秒走多少个完整周期"，而一个周期里只有一部分时间在支撑，
    // 所以必须除以 duty，才是支撑相真正持续多久。
    const double frequency_hz = std::max(config_.frequency_hz, 0.01);
    const double stance_duration_s = stance_duty(config_.pattern) / frequency_hz;

    // 速度 × 时间 = 位移：
    //   m/s   × s = m   （线速度 -> 步长，即这个支撑相内身体前进的距离）
    //   rad/s × s = rad （角速度 -> 转角）
    // 这是本层唯一一处把"速度命令"变成"位移"的地方，量纲必须自然成立。
    StepCommand command;
    command.translation = Eigen::Vector3d(
        body_twist.linear.x() * stance_duration_s,
        body_twist.linear.y() * stance_duration_s,
        0.0);
    command.turn_rad = body_twist.angular.z() * stance_duration_s;

    // 夹到配置上限。vx/vy/wz 来自外部输入，可能远超腿的机械能力；
    // 在轨迹层就拦住，比让 IK 解不出来更早、也更容易定位问题。
    command.translation.x() =
        std::clamp(command.translation.x(), -config_.step_length_m, config_.step_length_m);
    command.translation.y() =
        std::clamp(command.translation.y(), -config_.lateral_step_m, config_.lateral_step_m);
    command.turn_rad = std::clamp(command.turn_rad, -config_.turn_step_rad, config_.turn_step_rad);

    return command;
}

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

    // 支撑相的物理前提：脚掌与地面接触，在世界坐标系里保持不动。
    // 身体向前走 => 在身体坐标系里看，足端向后退（x 递减），
    // 后退的总量就是这个支撑相的"步长"。
    //
    // centered 在 fraction=0 时为 +1，在 fraction=0.5 时为 0，在 fraction=1 时为 -1；
    // 乘上"半步长"后，足端从 (nominal + 半步长) 走到 (nominal - 半步长)，
    // 总行程 = 一个完整步长，方向为"向后"。
    //
    // 符号很关键：这里的 "+ planar_motion" 必须和摆动相配合，才能满足衔接条件
    //   支撑相结束 (nominal - half) == 摆动相开始 (nominal - half)
    //   摆动相结束 (nominal + half) == 支撑相开始 (nominal + half)
    // 若写成减号，足端会在支撑相里向前移，并且**每个相位切换点**都会出现
    // 一个整步长的瞬跳（不只是起步时跳）。
    const double centered = 1.0 - (2.0 * fraction);
    const StepCommand step = compute_step_command(body_twist);
    const Eigen::Vector3d planar_motion = 0.5 * step.translation * centered;
    const double turn = 0.5 * step.turn_rad * centered;

    // 线速度产生平移目标，角速度产生绕身体 z 轴的平面转动目标。
    Eigen::Vector3d target = nominal + planar_motion;
    target = apply_planar_turn(target, turn);
    return target;
}

Eigen::Vector3d FootTrajectory::swing_trajectory(
    LegId leg_id,
    const Eigen::Vector3d& nominal,
    double fraction,
    const BodyTwist& body_twist) {
    (void)leg_id;

    // 摆动相分为：从 nominal 后方开始 -> 向前摆 -> 回到 nominal 前方。
    // progress 范围 [-1, +1]，与支撑相的 centered 对称：
    // 摆动相终点 = 支撑相起点，两相在 nominal 两侧自然衔接，不会跳变。
    const double progress = 2.0 * fraction - 1.0;
    // sin(pi*fraction) 在相位起止点为 0、中点为 1，形成平滑的抬脚高度。
    const double lift = std::sin(kPi * std::clamp(fraction, 0.0, 1.0));

    const StepCommand step = compute_step_command(body_twist);
    const Eigen::Vector3d planar_motion = 0.5 * step.translation * progress;
    const double turn = 0.5 * step.turn_rad * progress;

    Eigen::Vector3d target = nominal + planar_motion;
    target = apply_planar_turn(target, turn);
    target.z() += config_.step_height_m * lift;
    return target;
}

}  // namespace leg_calc
