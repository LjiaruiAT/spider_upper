#include "leg_calc/velocity_smoother.hpp"

#include <cmath>
#include <limits>

namespace leg_calc {

VelocitySmoother::VelocitySmoother(const GaitConfig& config) : config_(config) {}

double VelocitySmoother::approach(double current, double target, double max_delta) {
    const double delta = target - current;
    if (std::fabs(delta) <= max_delta) {
        return target;
    }
    return current + (delta > 0.0 ? max_delta : -max_delta);
}

BodyTwist VelocitySmoother::update(const BodyTwist& target, double dt) {
    if (dt <= 0.0) {
        // 时间没有推进，速度就不该变。注意返回的是上一状态而不是 target：
        // 否则一个 dt = 0 的调用就会把限幅整个绕过去。
        return current_;
    }

    // 上限 <= 0 表示"不做平滑"：用无穷大让步长限制失效，等价于直接跟随命令。
    // 保留这个开关是为了能一键回到"命令即输出"，方便做对照实验。
    constexpr double kNoLimit = std::numeric_limits<double>::infinity();
    const double max_linear_delta =
        config_.max_linear_accel_mps2 > 0.0 ? config_.max_linear_accel_mps2 * dt : kNoLimit;
    const double max_angular_delta =
        config_.max_angular_accel_rps2 > 0.0 ? config_.max_angular_accel_rps2 * dt : kNoLimit;

    // 只平滑平面运动真正用到的三个分量：x / y 平移与绕 z 转角。
    // 链上其余分量（z、roll、pitch）当前不产生，也不该被这里改写。
    current_.linear.x() = approach(current_.linear.x(), target.linear.x(), max_linear_delta);
    current_.linear.y() = approach(current_.linear.y(), target.linear.y(), max_linear_delta);
    current_.angular.z() = approach(current_.angular.z(), target.angular.z(), max_angular_delta);

    return current_;
}

}  // namespace leg_calc
