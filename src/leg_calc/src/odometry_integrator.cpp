#include "leg_calc/odometry_integrator.hpp"

#include <cmath>

namespace leg_calc {

namespace {
constexpr double kNegligibleAngularRate = 1e-9;

}  // namespace

void OdometryIntegrator::update(const BodyTwist& scaled_twist, double dt) {
    if (!(dt > 0.0)) {
        return;
    }

    const double vx = scaled_twist.linear.x();
    const double vy = scaled_twist.linear.y();
    const double w = scaled_twist.angular.z();

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();

    if (std::fabs(w) < kNegligibleAngularRate) {
        delta.translation() = Eigen::Vector3d(vx * dt, vy * dt, 0.0);
    } else {
        const Eigen::Vector2d icr(-vy / w, vx / w);
        const double dtheta = w * dt;
        const double c = std::cos(dtheta);
        const double s = std::sin(dtheta);
        const Eigen::Vector2d rotated_icr(c * icr.x() - s * icr.y(), s * icr.x() + c * icr.y());
        const Eigen::Vector2d translation = icr - rotated_icr;

        delta.translation() = Eigen::Vector3d(translation.x(), translation.y(), 0.0);
        delta.linear() = Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    odom_T_base_ = odom_T_base_ * delta;
}

void OdometryIntegrator::reset() {
    odom_T_base_ = Eigen::Isometry3d::Identity();
}

}  // namespace leg_calc
