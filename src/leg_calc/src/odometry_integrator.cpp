#include "leg_calc/odometry_integrator.hpp"

#include <cmath>

namespace leg_calc {

namespace {

// 小于这个角速度就当作"纯平移"。
//
// 为什么需要这个分支：圆弧积分要用到瞬时旋转中心 r_icr ∝ 1/ω，
// ω → 0 时 r_icr → ∞，而平移部分要算 (r_icr − Rz(dθ)·r_icr)——
// 两个巨大的数相减，有效位数会被吃光。
constexpr double kNegligibleAngularRate = 1e-9;

}  // namespace

void OdometryIntegrator::update(const BodyTwist& scaled_twist, double dt) {
    // dt <= 0 直接忽略：积分而已，没必要因为一次异常的时钟跳变就让位姿变成
    // NaN——那会让 RViz 里整棵 TF 树消失。
    if (!(dt > 0.0)) {
        return;
    }

    const double vx = scaled_twist.linear.x();
    const double vy = scaled_twist.linear.y();
    const double w = scaled_twist.angular.z();

    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();

    if (std::fabs(w) < kNegligibleAngularRate) {
        // 纯平移。这也是**直行**走的那条路——所以直行没有任何积分误差。
        delta.translation() = Eigen::Vector3d(vx * dt, vy * dt, 0.0);
    } else {
        // 匀速运动在 dt 内的精确解是**绕瞬时旋转中心转 w·dt 的一段圆弧**，
        // 不是"先直着走一段再转一点"。
        //
        // 为什么不用更简单的显式欧拉（p += v·dt; yaw += w·dt）：
        // 那个的半径每转一圈会外扩 exp(π·w·dt) 倍。以 wz=0.4、dt=20ms 算，
        // 每圈 +2.5%，转四圈就飘 10%——而且直行时完全看不出来，
        // 一到转弯就"莫名其妙地越转越大"，很难判断是步态问题还是显示问题。
        //
        // 圆弧积分对**匀速**输入是解析解，没有离散化误差：绕圈能精确闭合。
        //
        // 瞬时旋转中心在本体系的坐标（由 v = ω ẑ × (0 − r_icr) 反解）：
        //     r_icr = (−vy/ω,  vx/ω)
        // 直觉校验：vx>0、ω>0（左转）时 r_icr 在 +y 方向，也就是行进方向的左侧。
        const Eigen::Vector2d icr(-vy / w, vx / w);
        const double dtheta = w * dt;
        const double c = std::cos(dtheta);
        const double s = std::sin(dtheta);

        // 圆弧变换：p_new = r_icr + Rz(dθ)·(p − r_icr)
        //           = Rz(dθ)·p + (r_icr − Rz(dθ)·r_icr)
        // 括号里那项就是这个刚体变换的平移分量。
        const Eigen::Vector2d rotated_icr(c * icr.x() - s * icr.y(), s * icr.x() + c * icr.y());
        const Eigen::Vector2d translation = icr - rotated_icr;

        delta.translation() = Eigen::Vector3d(translation.x(), translation.y(), 0.0);
        delta.linear() = Eigen::AngleAxisd(dtheta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }

    // twist 是**本体坐标系**下的速度，所以增量直接**右乘**：
    //     odom_T_base_new = odom_T_base_old * T_delta
    // 右乘的物理含义就是"在本体坐标系里走了这么一段"，不需要手动去转当前航向角
    // ——位姿矩阵自己会把本体系的增量带到里程计系里去。
    odom_T_base_ = odom_T_base_ * delta;
}

void OdometryIntegrator::reset() {
    odom_T_base_ = Eigen::Isometry3d::Identity();
}

}  // namespace leg_calc
