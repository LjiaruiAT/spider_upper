// 手动调试节点：绕过数学链，直接往 /spider/servo_target 写 18 路舵机角。
//
// 用途（联调的第一步）：
//   · 实物接好后先发中位，确认所有舵机能通电、都停在 90.0° 附近；
//   · 逐路 / 逐腿给角度，核对"腿序 ↔ 通道"的假设与舵机安装方向；
//   · 标定时把指定的几路摆到测量需要的角度。
//
// 数据源：spider 包的 config/default_pose.yaml（启动时的初值，当前是全 900）。
// 交互方式：服务 /spider/manual_servo/set_angles，可一次设多路——
//           单腿调试就一次调 3 路（coxa / femur / tibia）。
//
// ⚠ **不要和 leg_calc 同时运行**：两者往同一个 /spider/servo_target 发布，
//    driver 收到的是两路帧的交错，真机上就是两个控制源打架。
//    （两个节点启动时都会检查并打 WARN，但这条注释才是最可靠的那道防线。）
//
// 为什么 1Hz 重发当前值，而不是"只在改动时发一次"：
//   driver 可能比本节点晚启动、或调试中被重启，一次性发布的帧会丢。
//   重发的是**内存里的当前值**（不是重读文件），所以不会覆盖你刚设的角度；
//   定时器回调刻意不打日志，避免把真正重要的信息刷掉。

#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_interfaces/msg/servo18.hpp"
#include "robot_interfaces/srv/set_servo_angles.hpp"

namespace {

constexpr std::size_t kServoChannelCount = 18;
// 与 Servo18Mapper / 下位机 ClampAngleDdeg 一致的合法区间（单位 0.1°）。
constexpr int kServoMinDdeg = 0;
constexpr int kServoMaxDdeg = 1800;

// 重发周期：只为"driver 晚启动 / 被重启"兜底，不需要快。
constexpr double kRepublishPeriodSec = 1.0;

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
}

// 读 default_pose.yaml 里的 `angle_ddeg:` 列表（形如 "  - 900"）。
// 跟 leg_layout / servo18_mapper 一样手写解析：这种一层的小 YAML 不值得引入 yaml-cpp。
std::array<int16_t, kServoChannelCount> load_default_pose(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    if (!input.is_open()) {
        throw std::runtime_error("手动调试：打不开默认姿态文件 " + yaml_path);
    }

    std::array<int16_t, kServoChannelCount> angles{};
    std::size_t count = 0;
    bool inside_list = false;

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        if (trimmed == "angle_ddeg:") {
            inside_list = true;
            continue;
        }
        if (!inside_list || trimmed.rfind("- ", 0) != 0) {
            continue;
        }

        if (count >= kServoChannelCount) {
            throw std::runtime_error("默认姿态文件里有超过 18 个角度：" + yaml_path);
        }
        const int angle = std::stoi(trim(trimmed.substr(2)));
        if (angle < kServoMinDdeg || angle > kServoMaxDdeg) {
            throw std::runtime_error(
                "默认姿态文件第 " + std::to_string(count) + " 路角度越界：" +
                std::to_string(angle) + "（合法区间 [0, 1800]，单位 0.1°）");
        }
        angles[count++] = static_cast<int16_t>(angle);
    }

    if (count != kServoChannelCount) {
        throw std::runtime_error(
            "默认姿态文件必须正好 18 个角度，实际 " + std::to_string(count) +
            " 个：" + yaml_path);
    }
    return angles;
}

}  // namespace

class ManualServoNode : public rclcpp::Node {
public:
    ManualServoNode() : Node("manual_servo_node") {
        const auto spider_share = ament_index_cpp::get_package_share_directory("spider");
        default_pose_path_ = spider_share + "/config/default_pose.yaml";
        angles_ = load_default_pose(default_pose_path_);

        // 检查 topic 上是否已经有别的发布者（典型：leg_calc 还在跑）。
        // 必须在自己创建 publisher **之前**查，否则会把自己也算进去。
        if (this->count_publishers("/spider/servo_target") > 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "/spider/servo_target 上已有其他发布者（可能是 leg_calc 仍在运行）。"
                "两个发布者会让 driver 收到两路帧的交错——请先停掉 leg_calc 再用手动调试。");
        }

        servo_target_publisher_ =
            this->create_publisher<robot_interfaces::msg::Servo18>("/spider/servo_target", 10);

        set_angles_service_ = this->create_service<robot_interfaces::srv::SetServoAngles>(
            "/spider/manual_servo/set_angles",
            std::bind(
                &ManualServoNode::handle_set_angles,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        republish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(kRepublishPeriodSec),
            [this]() { publish_current("republish", false); });

        RCLCPP_INFO(this->get_logger(), "manual_servo_node started（手动调试模式，绕过数学链）");
        RCLCPP_INFO(
            this->get_logger(),
            "已从 %s 载入 18 路初始角度并发送第一帧（900 = 90.0°）；之后 1Hz 重发当前值",
            default_pose_path_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "设置角度示例（单腿三路一起设）：ros2 service call /spider/manual_servo/set_angles "
            "robot_interfaces/srv/SetServoAngles \"{channels: [0, 1, 2], angles: [900, 800, 900]}\"");
        RCLCPP_WARN(
            this->get_logger(),
            "手动调试不要和 leg_calc 同时运行（会互相覆盖 /spider/servo_target）");
    }

private:
    // 发布当前缓存的 18 路角度。
    // log_event=true 时打一条 INFO（启动、服务调用各一条）；定时重发传 false，保持日志干净。
    void publish_current(const std::string& tag, bool log_event) {
        robot_interfaces::msg::Servo18 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "spider_base";
        msg.seq = sequence_++;
        msg.angle_ddeg = angles_;
        servo_target_publisher_->publish(msg);

        if (!log_event) {
            return;
        }
        std::string dump = "[";
        for (std::size_t i = 0; i < angles_.size(); ++i) {
            dump += std::to_string(angles_[i]);
            if (i + 1 < angles_.size()) {
                dump += ", ";
            }
        }
        dump += "]";
        RCLCPP_INFO(this->get_logger(), "[%s] 已发送 18 路：seq=%u, %s", tag.c_str(), msg.seq, dump.c_str());
    }

    void handle_set_angles(
        const std::shared_ptr<robot_interfaces::srv::SetServoAngles::Request> request,
        std::shared_ptr<robot_interfaces::srv::SetServoAngles::Response> response) {
        // 先全部校验、再全部写入：要么全生效、要么一处都不动。
        // "改了一半"的中间状态在调试时最难排查——你看到的是"某几路动了"。
        if (request->channels.empty()) {
            response->success = false;
            response->message = "channels 为空：至少设置 1 路";
            return;
        }
        if (request->channels.size() != request->angles.size()) {
            response->success = false;
            response->message = "channels 与 angles 长度不一致";
            return;
        }

        for (std::size_t i = 0; i < request->channels.size(); ++i) {
            const auto channel = request->channels[i];
            const auto angle = request->angles[i];
            if (channel >= kServoChannelCount) {
                response->success = false;
                response->message = "通道越界：" + std::to_string(channel) + "（合法 0~17）";
                return;
            }
            if (angle < kServoMinDdeg || angle > kServoMaxDdeg) {
                // 手动调试**拒绝**越界值而不是夹取：
                // 夹取会让"我设了 2000"和"舵机停在 1800"看起来一模一样。
                response->success = false;
                response->message = "第 " + std::to_string(i) + " 路角度越界：" +
                                    std::to_string(angle) + "（合法 [0, 1800]，单位 0.1°）";
                return;
            }
        }

        std::string changed;
        for (std::size_t i = 0; i < request->channels.size(); ++i) {
            const auto channel = request->channels[i];
            angles_[channel] = request->angles[i];
            if (!changed.empty()) {
                changed += "; ";
            }
            changed += "ch" + std::to_string(channel) + "=" + std::to_string(request->angles[i]);
        }

        publish_current("set_angles", true);
        response->success = true;
        response->message = "已设置并发送：" + changed;
    }

    std::string default_pose_path_;
    std::array<int16_t, kServoChannelCount> angles_{};
    uint8_t sequence_{0};
    rclcpp::Publisher<robot_interfaces::msg::Servo18>::SharedPtr servo_target_publisher_;
    rclcpp::Service<robot_interfaces::srv::SetServoAngles>::SharedPtr set_angles_service_;
    rclcpp::TimerBase::SharedPtr republish_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualServoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
