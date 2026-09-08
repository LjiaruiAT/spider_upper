#include "leg_calc/foot_trajectory.hpp"

#include <cmath>

namespace leg_calc {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d apply_planar_turn(const Eigen::Vector3d& nominal, double turn) {
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

    const double centered = 1.0 - (2.0 * fraction);
    const Eigen::Vector3d planar_motion(
        body_twist.linear.x() * step_half_ * centered,
        body_twist.linear.y() * step_half_ * centered,
        0.0);

    const double turn = body_twist.angular.z() * config_.turn_step_rad * centered;
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

    const double progress = 2.0 * fraction - 1.0;
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
