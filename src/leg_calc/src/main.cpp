#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/servo18.hpp>

#include "leg_calc/leg_kinematics.hpp"
#include "leg_calc/servo18_mapper.hpp"

namespace {

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
}

struct StaticLayoutConfig {
    double body_height_m{0.12};
    double left_y_m{0.12};
    double right_y_m{-0.12};
    double front_x_m{0.18};
    double middle_x_m{0.0};
    double rear_x_m{-0.18};
};

StaticLayoutConfig load_leg_layout_config(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open leg params file: " + yaml_path);
    }

    StaticLayoutConfig config;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        auto parse_mm_value = [&](const std::string& key, double& target) {
            if (trimmed.rfind(key, 0) == 0) {
                target = std::stod(trim(trimmed.substr(key.size()))) / 1000.0;
                return true;
            }
            return false;
        };

        if (trimmed == "leg_params:") {
            continue;
        }
        if (parse_mm_value("body_height_mm:", config.body_height_m)) {
            continue;
        }
        if (parse_mm_value("default_left_y_mm:", config.left_y_m)) {
            continue;
        }
        if (parse_mm_value("default_right_y_mm:", config.right_y_m)) {
            continue;
        }
        if (parse_mm_value("front_x_mm:", config.front_x_m)) {
            continue;
        }
        if (parse_mm_value("middle_x_mm:", config.middle_x_m)) {
            continue;
        }
        if (parse_mm_value("rear_x_mm:", config.rear_x_m)) {
            continue;
        }
    }

    return config;
}

const char* leg_name_cstr(leg_calc::LegId leg_id) {
    switch (leg_id) {
    case leg_calc::LegId::LeftFront:
        return "lf";
    case leg_calc::LegId::LeftMiddle:
        return "lm";
    case leg_calc::LegId::LeftRear:
        return "lr";
    case leg_calc::LegId::RightFront:
        return "rf";
    case leg_calc::LegId::RightMiddle:
        return "rm";
    case leg_calc::LegId::RightRear:
        return "rr";
    default:
        return "unknown";
    }
}

}  // namespace

// 这是 leg_calc 的当前教学型数学核心节点：
// 在保留单腿 IK/FK 验证价值的基础上，
// 当前已经开始承担正式主链中的“数学层输出者”角色。
// 当前会：
// 1. 从 leg_params.yaml 读取最小静态布局参数
// 2. 组织六条腿的默认足端目标点
// 3. 逐条腿做 IK，汇总成 SpiderJointTargets
// 4. 映射到 Servo18 兼容的 18 路数组
// 5. 正式发布 /spider/servo_target 给 robot_driver
class LegCalcNode : public rclcpp::Node {
public:
    LegCalcNode()
        : Node("leg_calc_node"), sequence_(0) {
        RCLCPP_INFO(this->get_logger(), "leg_calc_node started");
        RCLCPP_INFO(this->get_logger(), "Current stage: hexapod static organization skeleton + formal Servo18 output");

        task_cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spider/task_cmd_vel",
            10,
            std::bind(&LegCalcNode::task_cmd_vel_callback, this, std::placeholders::_1));

        servo_target_publisher_ = this->create_publisher<robot_interfaces::msg::Servo18>("/spider/servo_target", 10);

        demo_chain_ = build_demo_chain();
        kinematics_ = std::make_shared<leg_calc::LegKinematics>(demo_chain_);
        kinematics_->set_position_offset(Eigen::Vector3d(0.0, 0.0, 0.0));

        servo_map_path_ = "/home/liujiarui/Desktop/spider_upper/src/spider/config/servo_map.yaml";
        leg_params_path_ = "/home/liujiarui/Desktop/spider_upper/src/spider/config/leg_params.yaml";

        layout_config_ = load_leg_layout_config(leg_params_path_);
        servo_map_ = leg_calc::Servo18Mapper::load_map_from_yaml(servo_map_path_);

        RCLCPP_INFO(this->get_logger(), "Loaded leg layout from %s", leg_params_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Loaded servo_map from %s", servo_map_path_.c_str());

        publish_servo_target("neutral");
    }

private:
    leg_calc::SpiderFootTargets build_static_foot_targets(const geometry_msgs::msg::Twist& task_cmd_vel) const {
        leg_calc::SpiderFootTargets targets;

        const double x_shift = task_cmd_vel.linear.x * 0.05;
        const double y_shift = task_cmd_vel.linear.y * 0.05;
        const double turn_hint = task_cmd_vel.angular.z * 0.02;

        for (const auto leg_id : leg_calc::kAllLegIds) {
            double x = layout_x_for_leg(leg_id) + x_shift;
            double y = layout_y_for_leg(leg_id) + y_shift;

            if (leg_id == leg_calc::LegId::LeftFront || leg_id == leg_calc::LegId::RightFront) {
                x += turn_hint;
            } else if (leg_id == leg_calc::LegId::LeftRear || leg_id == leg_calc::LegId::RightRear) {
                x -= turn_hint;
            }

            targets.feet[leg_calc::leg_index(leg_id)] = Eigen::Vector3d(x, y, -layout_config_.body_height_m);
        }

        return targets;
    }

    leg_calc::SpiderJointTargets solve_static_joint_targets(
        const leg_calc::SpiderFootTargets& foot_targets,
        const std::string& tag) {
        leg_calc::SpiderJointTargets spider_targets;

        for (const auto leg_id : leg_calc::kAllLegIds) {
            int ik_result = -1;
            const Eigen::Vector3d& foot_target = foot_targets.feet[leg_calc::leg_index(leg_id)];
            const auto joint_solution = kinematics_->inverse_position(foot_target, &ik_result);
            const auto reconstructed_position = kinematics_->forward_position(joint_solution);

            spider_targets.legs[leg_calc::leg_index(leg_id)].joints = joint_solution;

            RCLCPP_INFO(
                this->get_logger(),
                "[%s] leg=%s target=[%.4f, %.4f, %.4f] ik=%d joints=[%.4f, %.4f, %.4f] fk=[%.4f, %.4f, %.4f]",
                tag.c_str(),
                leg_name_cstr(leg_id),
                foot_target.x(),
                foot_target.y(),
                foot_target.z(),
                ik_result,
                joint_solution(0),
                joint_solution(1),
                joint_solution(2),
                reconstructed_position.x(),
                reconstructed_position.y(),
                reconstructed_position.z());
        }

        return spider_targets;
    }

    void publish_servo_target(const std::string& tag) {
        const auto foot_targets = build_static_foot_targets(latest_task_cmd_vel_);
        const auto spider_targets = solve_static_joint_targets(foot_targets, tag);
        const auto servo_angles = leg_calc::Servo18Mapper::to_angle_ddeg(spider_targets, servo_map_);

        robot_interfaces::msg::Servo18 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "spider_base";
        msg.seq = sequence_++;
        msg.angle_ddeg = servo_angles;
        servo_target_publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "[%s] task_cmd_vel=(%.3f, %.3f, %.3f)",
            tag.c_str(),
            latest_task_cmd_vel_.linear.x,
            latest_task_cmd_vel_.linear.y,
            latest_task_cmd_vel_.angular.z);
        RCLCPP_INFO(
            this->get_logger(),
            "[%s] static layout body_height=%.3f m, left_y=%.3f m, right_y=%.3f m, front/mid/rear x=[%.3f, %.3f, %.3f] m",
            tag.c_str(),
            layout_config_.body_height_m,
            layout_config_.left_y_m,
            layout_config_.right_y_m,
            layout_config_.front_x_m,
            layout_config_.middle_x_m,
            layout_config_.rear_x_m);
        RCLCPP_INFO(this->get_logger(), "[%s] Publishing Servo18 angle_ddeg = %s", tag.c_str(), leg_calc::Servo18Mapper::to_debug_string(servo_angles).c_str());
    }

    void task_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_task_cmd_vel_ = *msg;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received /spider/task_cmd_vel in leg_calc: vx=%.3f, vy=%.3f, wz=%.3f",
            latest_task_cmd_vel_.linear.x,
            latest_task_cmd_vel_.linear.y,
            latest_task_cmd_vel_.angular.z);

        publish_servo_target("task_cmd_vel");
    }

    double layout_x_for_leg(leg_calc::LegId leg_id) const {
        switch (leg_id) {
        case leg_calc::LegId::LeftFront:
        case leg_calc::LegId::RightFront:
            return layout_config_.front_x_m;
        case leg_calc::LegId::LeftMiddle:
        case leg_calc::LegId::RightMiddle:
            return layout_config_.middle_x_m;
        case leg_calc::LegId::LeftRear:
        case leg_calc::LegId::RightRear:
            return layout_config_.rear_x_m;
        default:
            return 0.0;
        }
    }

    double layout_y_for_leg(leg_calc::LegId leg_id) const {
        return leg_calc::is_left_leg(leg_id) ? layout_config_.left_y_m : layout_config_.right_y_m;
    }

    static KDL::Chain build_demo_chain() {
        KDL::Chain chain;
        chain.addSegment(KDL::Segment(
            "joint1",
            KDL::Joint(KDL::Joint::RotZ),
            KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));
        chain.addSegment(KDL::Segment(
            "joint2",
            KDL::Joint(KDL::Joint::RotY),
            KDL::Frame(KDL::Vector(0.06, 0.0, 0.0))));
        chain.addSegment(KDL::Segment(
            "joint3",
            KDL::Joint(KDL::Joint::RotY),
            KDL::Frame(KDL::Vector(0.12, 0.0, -0.02))));
        chain.addSegment(KDL::Segment(
            "foot",
            KDL::Joint(KDL::Joint::None),
            KDL::Frame(KDL::Vector(0.10, 0.0, -0.10))));
        return chain;
    }

    uint8_t sequence_;
    geometry_msgs::msg::Twist latest_task_cmd_vel_{};
    StaticLayoutConfig layout_config_{};
    KDL::Chain demo_chain_;
    std::shared_ptr<leg_calc::LegKinematics> kinematics_;
    std::vector<leg_calc::ServoMapEntry> servo_map_{};
    std::string servo_map_path_;
    std::string leg_params_path_;
    rclcpp::Publisher<robot_interfaces::msg::Servo18>::SharedPtr servo_target_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr task_cmd_vel_subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegCalcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
