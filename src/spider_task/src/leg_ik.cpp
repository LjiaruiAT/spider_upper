// 单腿 IK 控制节点：输入腿局部坐标 (x, y, z)，让指定腿的足端走到那个点。
//
// 与 manual_servo_node 的分工：
//   manual_servo_node —— 直接给 18 路舵机角（不含运动学）
//   leg_ik_node       —— 给"腿局部坐标"，内部 IK 解算成关节角，再映射到该腿的 3 路
//
// 数据流：
//   服务 (leg, x, y, z) [mm]
//     → 目标点（腿局部系，m）
//     → IK  → q = [coxa, femur, tibia] (rad)
//     → 三道检查：IK 收敛 / FK 回代误差 / 关节限位
//     → 写进该腿的关节目标（其余五条腿保持不变）
//     → Servo18Mapper → 18 路 0.1°
//     → 20ms 定时器按 1 秒 quintic 插值推进 → 发布 /spider/servo_target
//
// 腿局部坐标系（见 leg_params.yaml）：原点在髋轴，x 沿 coxa 朝外，z 向上。
// 六条腿共用一套坐标 —— 同一个 (x, y, z) 对任何一条腿都成立。
//
// ⚠ **不要和 leg_calc 同时运行**：两者都往 /spider/servo_target 发布。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chain.hpp>
#include <rclcpp/rclcpp.hpp>

#include "leg_calc/common_types.hpp"
#include "leg_calc/leg_chain.hpp"
#include "leg_calc/leg_kinematics.hpp"
#include "leg_calc/leg_layout.hpp"
#include "leg_calc/servo18_mapper.hpp"
#include "robot_interfaces/msg/servo18.hpp"
#include "robot_interfaces/srv/leg_move_to.hpp"

namespace {

constexpr std::size_t kServoChannelCount = 18;

// IK 回代误差容差（米）：与 leg_calc 一致，0.1mm。
constexpr double kIkErrorToleranceM = 1e-4;

// 平滑时长（秒）与发布周期（秒）。不平滑会让舵机全速猛冲。
constexpr double kSmoothDurationSec = 1.0;
constexpr double kControlPeriodSec = 0.02;

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
}

// 关节限位配置（度），读法与 leg_calc/src/main.cpp 保持一致。
struct JointLimitsConfig {
    double coxa_min_deg{-90.0};
    double coxa_max_deg{90.0};
    double femur_min_deg{-60.0};
    double femur_max_deg{60.0};
    double tibia_min_deg{-30.0};
    double tibia_max_deg{150.0};
};

JointLimitsConfig load_joint_limits_config(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    if (!input.is_open()) {
        throw std::runtime_error("打不开腿参数文件: " + yaml_path);
    }

    JointLimitsConfig config;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        auto parse_deg = [&](const std::string& key, double& target) {
            if (trimmed.rfind(key, 0) == 0) {
                target = std::stod(trim(trimmed.substr(key.size())));
                return true;
            }
            return false;
        };

        if (trimmed == "leg_params:") {
            continue;
        }
        if (parse_deg("coxa_min_deg:", config.coxa_min_deg)) continue;
        if (parse_deg("coxa_max_deg:", config.coxa_max_deg)) continue;
        if (parse_deg("femur_min_deg:", config.femur_min_deg)) continue;
        if (parse_deg("femur_max_deg:", config.femur_max_deg)) continue;
        if (parse_deg("tibia_min_deg:", config.tibia_min_deg)) continue;
        if (parse_deg("tibia_max_deg:", config.tibia_max_deg)) continue;
    }
    return config;
}

leg_calc::JointLimits to_joint_limits(const JointLimitsConfig& config) {
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    leg_calc::JointLimits limits;
    limits.min = leg_calc::JointVector(
        config.coxa_min_deg * kDegToRad,
        config.femur_min_deg * kDegToRad,
        config.tibia_min_deg * kDegToRad);
    limits.max = leg_calc::JointVector(
        config.coxa_max_deg * kDegToRad,
        config.femur_max_deg * kDegToRad,
        config.tibia_max_deg * kDegToRad);
    return limits;
}

// 五阶多项式缓动：t=0/1 处速度与加速度都为 0（起步、到位都不顿）。
double quintic_ease(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// 腿名 → LegId；非法名字抛异常。
leg_calc::LegId parse_leg_id(const std::string& name) {
    for (const auto leg_id : leg_calc::kAllLegIds) {
        if (leg_calc::leg_name(leg_id) == name) {
            return leg_id;
        }
    }
    throw std::runtime_error("未知腿名: '" + name + "'（合法：lf / lm / lr / rf / rm / rr）");
}

}  // namespace

class LegIkNode : public rclcpp::Node {
public:
    LegIkNode()
        : Node("leg_ik_node") {
        const auto spider_share = ament_index_cpp::get_package_share_directory("spider");
        const std::string leg_params_path = spider_share + "/config/leg_params.yaml";
        const std::string servo_map_path = spider_share + "/config/servo_map.yaml";

        // ---- 布局与运动学（六条腿共用同一条链：输入是腿局部坐标）----
        layout_ = leg_calc::load_leg_layout_from_yaml(leg_params_path);
        const KDL::Chain chain = leg_calc::build_leg_chain(
            layout_.coxa_length_m, layout_.femur_length_m, layout_.tibia_length_m);
        ik_ = std::make_unique<leg_calc::LegKinematics>(chain);

        limits_ = to_joint_limits(load_joint_limits_config(leg_params_path));
        servo_map_ = leg_calc::Servo18Mapper::load_map_from_yaml(servo_map_path);

        // ---- 初值：六条腿都取「站姿」（home_local_m）的解 ----
        const Eigen::Vector3d home = layout_.home_local_m;
        for (std::size_t i = 0; i < leg_calc::kLegCount; ++i) {
            int result = -1;
            const leg_calc::JointVector q = ik_->inverse_position(home, &result);
            if (result < 0 || !limits_.contains(q)) {
                throw std::runtime_error(
                    "站姿 IK 失败：leg_params.yaml 里的 home_local_mm 不可达或超出关节限位");
            }
            targets_.legs[i].joints = q;
        }
        rebuild_angles();
        start_ = angles_;
        goal_ = angles_;

        // ---- 检查是否已有其他发布者（典型：leg_calc 还在跑）----
        if (this->count_publishers("/spider/servo_target") > 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "/spider/servo_target 上已有其他发布者（可能是 leg_calc 仍在运行）。"
                "两者会互相覆盖——请先停掉 leg_calc。");
        }

        publisher_ =
            this->create_publisher<robot_interfaces::msg::Servo18>("/spider/servo_target", 10);

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(kControlPeriodSec), [this]() { on_timer(); });

        service_ = this->create_service<robot_interfaces::srv::LegMoveTo>(
            "/spider/leg_ik/move_to",
            std::bind(
                &LegIkNode::handle_move_to, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "leg_ik_node started（单腿 IK 控制，绕过步态层）");
        RCLCPP_INFO(
            this->get_logger(),
            "站姿参考点（腿局部系，mm）：x=%.1f  y=%.1f  z=%.1f",
            layout_.home_local_m.x() * 1000.0,
            layout_.home_local_m.y() * 1000.0,
            layout_.home_local_m.z() * 1000.0);
        RCLCPP_INFO(
            this->get_logger(),
            "用法示例：ros2 service call /spider/leg_ik/move_to robot_interfaces/srv/LegMoveTo "
            "\"{leg: 'lf', x: %.1f, y: 0.0, z: -120.0}\"",
            layout_.home_local_m.x() * 1000.0);
        RCLCPP_WARN(
            this->get_logger(),
            "不要和 leg_calc 同时运行（会互相覆盖 /spider/servo_target）");
    }

private:
    // 关节目标 → 18 路舵机角（含标定映射），并统计被夹取的通道数。
    void rebuild_angles() {
        const auto mapped = leg_calc::Servo18Mapper::to_angle_ddeg(targets_, servo_map_);
        angles_ = mapped.angle_ddeg;
        if (mapped.out_of_range_count > 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "映射后有 %zu 路角度被夹取到 [0, 1800]（当前未标定属正常）",
                mapped.out_of_range_count);
        }
    }

    // 20ms：按 quintic 插值推进，并发布。
    void on_timer() {
        double progress = 1.0;
        if (smooth_active_) {
            progress = (this->now() - smooth_start_).seconds() / kSmoothDurationSec;
            if (progress >= 1.0) {
                progress = 1.0;
                smooth_active_ = false;
            }
        }
        const double ease = quintic_ease(progress);

        robot_interfaces::msg::Servo18 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "spider_base";
        msg.seq = sequence_++;
        for (std::size_t i = 0; i < kServoChannelCount; ++i) {
            const double value =
                static_cast<double>(start_[i]) + (static_cast<double>(goal_[i]) - start_[i]) * ease;
            msg.angle_ddeg[i] = static_cast<int16_t>(std::lround(value));
        }
        publisher_->publish(msg);
    }

    void handle_move_to(
        const std::shared_ptr<robot_interfaces::srv::LegMoveTo::Request> request,
        std::shared_ptr<robot_interfaces::srv::LegMoveTo::Response> response) {
        // ---- 腿名 ----
        leg_calc::LegId leg_id;
        try {
            leg_id = parse_leg_id(request->leg);
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
            return;
        }

        // ---- 目标点：mm → m ----
        const Eigen::Vector3d target_m(request->x / 1000.0, request->y / 1000.0, request->z / 1000.0);

        // ---- IK + 三道检查 ----
        int ik_result = -1;
        const leg_calc::JointVector q = ik_->inverse_position(target_m, &ik_result);
        if (ik_result < 0) {
            response->success = false;
            char buf[160];
            std::snprintf(
                buf, sizeof(buf), "IK 未收敛（目标 [%.1f, %.1f, %.1f] mm）", request->x, request->y,
                request->z);
            response->message = buf;
            return;
        }

        const Eigen::Vector3d reconstructed = ik_->forward_position(q);
        const double error_mm = (reconstructed - target_m).norm() * 1000.0;
        if (error_mm > kIkErrorToleranceM * 1000.0) {
            response->success = false;
            char buf[200];
            std::snprintf(
                buf, sizeof(buf),
                "解不可信：FK 回代误差 %.2f mm（容差 %.2f mm）——目标多半不可达",
                error_mm, kIkErrorToleranceM * 1000.0);
            response->message = buf;
            return;
        }

        if (!limits_.contains(q)) {
            response->success = false;
            char buf[220];
            std::snprintf(
                buf, sizeof(buf),
                "超出关节限位：解 [%.1f, %.1f, %.1f]°，限位 coxa[%.0f,%.0f] femur[%.0f,%.0f] tibia[%.0f,%.0f]",
                q(0) * 180.0 / M_PI, q(1) * 180.0 / M_PI, q(2) * 180.0 / M_PI,
                limits_.min(0) * 180.0 / M_PI, limits_.max(0) * 180.0 / M_PI,
                limits_.min(1) * 180.0 / M_PI, limits_.max(1) * 180.0 / M_PI,
                limits_.min(2) * 180.0 / M_PI, limits_.max(2) * 180.0 / M_PI);
            response->message = buf;
            return;
        }

        // ---- 通过：只更新目标腿，其余五条腿保持 ----
        const std::size_t leg_index = leg_calc::leg_index(leg_id);
        targets_.legs[leg_index].joints = q;
        rebuild_angles();

        // 从**当前实际位置**开始插值（避免每次指令都从更早的位置重来）。
        start_ = current_output();
        goal_ = angles_;
        smooth_start_ = this->now();
        smooth_active_ = true;

        char buf[200];
        std::snprintf(
            buf, sizeof(buf),
            "已下发 %s：[coxa %.1f°, femur %.1f°, tibia %.1f°]（FK 误差 %.3f mm，1 秒内到位）",
            leg_calc::leg_name(leg_id).c_str(), q(0) * 180.0 / M_PI, q(1) * 180.0 / M_PI,
            q(2) * 180.0 / M_PI, error_mm);
        response->success = true;
        response->message = buf;

        RCLCPP_INFO(this->get_logger(), "%s", buf);
    }

    // 当前插值位置（18 路）。
    std::array<int16_t, kServoChannelCount> current_output() const {
        double progress = 1.0;
        if (smooth_active_) {
            progress = (this->now() - smooth_start_).seconds() / kSmoothDurationSec;
            if (progress >= 1.0) {
                progress = 1.0;
            }
        }
        const double ease = quintic_ease(progress);
        std::array<int16_t, kServoChannelCount> out{};
        for (std::size_t i = 0; i < kServoChannelCount; ++i) {
            out[i] = static_cast<int16_t>(std::lround(
                static_cast<double>(start_[i]) + (static_cast<double>(goal_[i]) - start_[i]) * ease));
        }
        return out;
    }

    // ---- 配置 ----
    leg_calc::LegLayoutConfig layout_{};
    leg_calc::JointLimits limits_{};
    std::vector<leg_calc::ServoMapEntry> servo_map_;

    // ---- 运动学与目标 ----
    std::unique_ptr<leg_calc::LegKinematics> ik_;
    leg_calc::SpiderJointTargets targets_{};
    std::array<int16_t, kServoChannelCount> angles_{};

    // ---- 插值状态 ----
    std::array<int16_t, kServoChannelCount> start_{};
    std::array<int16_t, kServoChannelCount> goal_{};
    bool smooth_active_{false};
    rclcpp::Time smooth_start_{0, 0, RCL_ROS_TIME};

    // ---- ROS ----
    uint8_t sequence_{0};
    rclcpp::Publisher<robot_interfaces::msg::Servo18>::SharedPtr publisher_;
    rclcpp::Service<robot_interfaces::srv::LegMoveTo>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegIkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
