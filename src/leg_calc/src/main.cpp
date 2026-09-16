#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/servo18.hpp>

#include "leg_calc/common_types.hpp"
#include "leg_calc/foot_trajectory.hpp"
#include "leg_calc/gait_phase_manager.hpp"
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

// IK 回代误差容差（米）。IK 求出关节角 q 后，用同一条 KDL 链做 FK 应回到目标点；
// 误差超过这个值说明"求解器返回了结果"但"结果没到目标"，属于必须报警的情况。
constexpr double kIkErrorToleranceM = 1e-4;  // 0.1 mm

bool is_zero_command(const geometry_msgs::msg::Twist& cmd) {
    constexpr double kEpsilon = 1e-6;
    return std::fabs(cmd.linear.x) < kEpsilon &&
           std::fabs(cmd.linear.y) < kEpsilon &&
           std::fabs(cmd.angular.z) < kEpsilon;
}

leg_calc::BodyTwist to_body_twist(const geometry_msgs::msg::Twist& cmd) {
    leg_calc::BodyTwist body_twist;
    body_twist.linear = Eigen::Vector3d(cmd.linear.x, cmd.linear.y, 0.0);
    body_twist.angular = Eigen::Vector3d(0.0, 0.0, cmd.angular.z);
    return body_twist;
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

// 这是 leg_calc 当前的数学装配节点。注意：IK 只是其中的“位置目标 -> 关节角”一步，
// 前面还有步态相位和足端轨迹，后面还有关节到 Servo18 通道的映射：
//
//   cmd_vel
//     -> gait phase
//     -> foot trajectory（身体坐标系足端目标）
//     -> body frame -> leg frame
//     -> single-leg IK
//     -> FK reconstruction（误差/一致性诊断）
//     -> Servo18 mapping
//
// 当前 demo 的 KDL 链和六条腿几何还未完全替换为真实机械参数，详见 build_demo_chain
// 和 solve_joint_targets 内的说明。
class LegCalcNode : public rclcpp::Node {
public:
    LegCalcNode()
        : Node("leg_calc_node"),
          sequence_(0),
          gait_config_(),
          gait_phase_manager_(gait_config_),
          foot_trajectory_(gait_config_) {
        RCLCPP_INFO(this->get_logger(), "leg_calc_node started");
        RCLCPP_INFO(this->get_logger(), "Current stage: gait phase -> foot trajectory -> body frame -> leg frame -> IK -> Servo18");

        declare_parameter<int>("control_period_ms", 20);
        control_period_ms_ = this->get_parameter("control_period_ms").as_int();
        if (control_period_ms_ <= 0) {
            control_period_ms_ = 20;
        }
        control_period_sec_ = static_cast<double>(control_period_ms_) / 1000.0;

        task_cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spider/task_cmd_vel",
            10,
            std::bind(&LegCalcNode::task_cmd_vel_callback, this, std::placeholders::_1));

        servo_target_publisher_ = this->create_publisher<robot_interfaces::msg::Servo18>("/spider/servo_target", 10);

        demo_chain_ = build_demo_chain();
        kinematics_ = std::make_shared<leg_calc::LegKinematics>(demo_chain_);
        kinematics_->set_position_offset(Eigen::Vector3d(0.0, 0.0, 0.0));

        const auto spider_share = ament_index_cpp::get_package_share_directory("spider");
        servo_map_path_ = spider_share + "/config/servo_map.yaml";
        leg_params_path_ = spider_share + "/config/leg_params.yaml";

        layout_config_ = load_leg_layout_config(leg_params_path_);
        servo_map_ = leg_calc::Servo18Mapper::load_map_from_yaml(servo_map_path_);
        frame_bundle_ = build_frame_bundle(layout_config_);

        foot_trajectory_.update_config(gait_config_);
        gait_phase_manager_.update_config(gait_config_);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms_),
            std::bind(&LegCalcNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Loaded leg layout from %s", leg_params_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Loaded servo_map from %s", servo_map_path_.c_str());

        publish_servo_target("neutral", build_nominal_body_targets());
    }

private:
    leg_calc::BodyFootTargets build_nominal_body_targets() const {
        // 中性姿态的目标足端位置统一定义在身体坐标系：
        //   x：前后，y：左右，z：身体下方为负（这里使用 -body_height_m）。
        // 这些目标还不是 IK 的输入；solve_joint_targets 会根据每条腿的安装位姿
        // 转换成相应的腿局部坐标，再交给同一个单腿运动学对象。
        leg_calc::BodyFootTargets targets;

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            targets.feet[index] = Eigen::Vector3d(
                layout_x_for_leg(leg_id),
                layout_y_for_leg(leg_id),
                -layout_config_.body_height_m);
        }

        return targets;
    }

    leg_calc::BodyFootTargets build_motion_body_targets(const leg_calc::BodyTwist& body_twist) {
        // 先从站立时的名义足端位置开始，再由当前相位和身体速度生成运动目标。
        // 因此 IK 每个周期看到的是一个随时间变化的身体坐标系点。
        leg_calc::BodyFootTargets targets = build_nominal_body_targets();
        const auto& gait_state = gait_phase_manager_.state();

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            const auto& nominal = targets.feet[index];
            targets.feet[index] = foot_trajectory_.compute_foot_target(
                leg_id,
                nominal,
                gait_state.phases[index],
                gait_state.phase_fraction[index],
                body_twist);
        }

        return targets;
    }

    void control_loop() {
        const auto latest_cmd = snapshot_latest_task_cmd_vel();
        const bool motion_active = !is_zero_command(latest_cmd);

        if (motion_active && !motion_active_) {
            gait_phase_manager_.reset();
        } else if (!motion_active && motion_active_) {
            gait_phase_manager_.reset();
        }
        motion_active_ = motion_active;

        if (!motion_active) {
            publish_servo_target("stand", build_nominal_body_targets());
            return;
        }

        gait_phase_manager_.tick(control_period_sec_);
        publish_servo_target("gait", build_motion_body_targets(to_body_twist(latest_cmd)));
    }

    geometry_msgs::msg::Twist snapshot_latest_task_cmd_vel() const {
        std::lock_guard<std::mutex> lock(task_cmd_mutex_);
        return latest_task_cmd_vel_;
    }

    leg_calc::SpiderJointTargets solve_joint_targets(
        const leg_calc::BodyFootTargets& body_foot_targets,
        const std::string& tag) {
        // 每条腿的解算路径：
        //   身体系目标 p_body
        //     -> p_leg = body_T_leg^{-1} p_body
        //     -> IK 求 q
        //     -> FK 得到 p_leg_reconstructed
        //     -> body_T_leg p_leg_reconstructed（仅用于日志诊断）
        //
        // 六条腿当前共用同一条 demo KDL chain；真正的机器人通常还需要根据
        // 左右侧镜像、腿座方向和每条腿实际尺寸建立正确的链。
        leg_calc::SpiderJointTargets spider_targets;

        // 回代诊断统计。solve_joint_targets() 每个控制周期都会被调用（默认 20ms），
        // 所以不能对每条腿各打一条 WARN，否则日志会被刷爆；这里先统计，循环结束后汇总一条。
        std::size_t ik_error_count = 0;
        std::size_t error_exceed_count = 0;
        double max_error_norm = 0.0;

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            const auto& body_target = body_foot_targets.feet[index];
            const auto& mount = frame_bundle_.leg_mounts[index];
            // KDL 链的根坐标系是这条腿的局部坐标系，所以身体系目标必须先逆变换。
            const auto leg_target = leg_calc::body_point_to_leg_point(body_target, mount);

            int ik_result = -1;
            const auto joint_solution = kinematics_->inverse_position(leg_target, &ik_result);
            // FK 回代不是 IK 的第二次求解，而是检查 q 经正运动学后是否回到了目标点。
            const auto reconstructed_position = kinematics_->forward_position(joint_solution);
            const auto reconstructed_body_position = leg_calc::leg_point_to_body_point(reconstructed_position, mount);

            // 回代误差在"腿坐标系"下比较，因为 IK 的目标点正是这个坐标系下的 leg_target。
            // LegKinematics 内部对 position_offset_ 的处理在 IK/FK 两边是对称的，
            // 所以这里直接用 reconstructed_position - leg_target，不需要再补偏移。
            // 当前只统计和报警，失败解仍然会被写入输出（拒绝策略是下一个主题）。
            const double error_norm = (reconstructed_position - leg_target).norm();
            max_error_norm = std::max(max_error_norm, error_norm);
            if (ik_result < 0) {
                ++ik_error_count;
            }
            if (error_norm > kIkErrorToleranceM) {
                ++error_exceed_count;
            }

            spider_targets.legs[index].joints = joint_solution;

            RCLCPP_DEBUG(
                this->get_logger(),
                "[%s] leg=%s body_target=[%.4f, %.4f, %.4f] leg_target=[%.4f, %.4f, %.4f] ik=%d err=%.2fmm joints=[%.4f, %.4f, %.4f] fk_body=[%.4f, %.4f, %.4f]",
                tag.c_str(),
                leg_name_cstr(leg_id),
                body_target.x(),
                body_target.y(),
                body_target.z(),
                leg_target.x(),
                leg_target.y(),
                leg_target.z(),
                ik_result,
                error_norm * 1000.0,
                joint_solution(0),
                joint_solution(1),
                joint_solution(2),
                reconstructed_body_position.x(),
                reconstructed_body_position.y(),
                reconstructed_body_position.z());
        }

        // 只要有腿 IK 报错、或有腿回代误差超出容差，就汇总打一条 WARN。
        // WARN_THROTTLE 保证 50Hz 控制循环下日志不会刷屏，但问题会持续可见。
        if (ik_error_count > 0 || error_exceed_count > 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[%s] IK 回代诊断异常: ik_result<0 的有 %zu/%zu 腿, FK 回代误差超过 %.2fmm 的有 %zu/%zu 腿, 最大误差=%.2f mm",
                tag.c_str(),
                ik_error_count,
                leg_calc::kLegCount,
                kIkErrorToleranceM * 1000.0,
                error_exceed_count,
                leg_calc::kLegCount,
                max_error_norm * 1000.0);
        } else {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[%s] IK 回代诊断正常: 最大误差=%.4f mm",
                tag.c_str(),
                max_error_norm * 1000.0);
        }

        return spider_targets;
    }

    void publish_servo_target(const std::string& tag, const leg_calc::BodyFootTargets& body_foot_targets) {
        const auto spider_targets = solve_joint_targets(body_foot_targets, tag);
        // 映射层输出的是"真实舵机角"（0~1800），标定参数来自 servo_map.yaml。
        const auto mapping = leg_calc::Servo18Mapper::to_angle_ddeg(spider_targets, servo_map_);
        const auto& servo_angles = mapping.angle_ddeg;

        // 越界说明 IK 解出的角度或标定参数有问题。在真机上，这就是"舵机顶死"，
        // 所以必须报出来，而不是让它静默地被夹取掉。
        if (mapping.out_of_range_count > 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[%s] 标定后有 %zu/%zu 路角度超出合法区间 [%d, %d]，已夹取；请检查 IK 结果或 servo_map 标定参数",
                tag.c_str(),
                mapping.out_of_range_count,
                leg_calc::Servo18Mapper::kServoChannelCount,
                leg_calc::Servo18Mapper::kServoMinDdeg,
                leg_calc::Servo18Mapper::kServoMaxDdeg);
        }

        robot_interfaces::msg::Servo18 msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_bundle_.body_frame.frame_id;
        msg.seq = sequence_++;
        msg.angle_ddeg = servo_angles;
        servo_target_publisher_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[%s] body_frame=%s, task_cmd_vel=(%.3f, %.3f, %.3f), Servo18=%s",
            tag.c_str(),
            frame_bundle_.body_frame.frame_id.c_str(),
            latest_task_cmd_vel_.linear.x,
            latest_task_cmd_vel_.linear.y,
            latest_task_cmd_vel_.angular.z,
            leg_calc::Servo18Mapper::to_debug_string(servo_angles).c_str());
    }

    void task_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(task_cmd_mutex_);
            latest_task_cmd_vel_ = *msg;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received /spider/task_cmd_vel in leg_calc: vx=%.3f, vy=%.3f, wz=%.3f",
            msg->linear.x,
            msg->linear.y,
            msg->angular.z);
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

    static leg_calc::SpiderFrameBundle build_frame_bundle(const StaticLayoutConfig& config) {
        leg_calc::SpiderFrameBundle bundle;
        bundle.body_frame.frame_id = "spider_base";

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            const Eigen::Vector3d origin(
                (leg_id == leg_calc::LegId::LeftFront || leg_id == leg_calc::LegId::RightFront) ? config.front_x_m
                : (leg_id == leg_calc::LegId::LeftMiddle || leg_id == leg_calc::LegId::RightMiddle) ? config.middle_x_m
                                                                                                 : config.rear_x_m,
                leg_calc::is_left_leg(leg_id) ? config.left_y_m : config.right_y_m,
                0.0);
            bundle.leg_mounts[index] = leg_calc::make_leg_mount_pose(leg_id, origin);
        }

        return bundle;
    }

    static KDL::Chain build_demo_chain() {
        // 这里只是为了让数学链路可以运行的演示模型，不是真实蜘蛛腿参数：
        //   joint1: RotZ  -> coxa/yaw，改变腿在水平面的方向
        //   joint2: RotY  -> femur/pitch
        //   joint3: RotY  -> tibia/knee
        // 后面的无关节 segment 只提供固定末端几何偏移。
        // 当前 leg_params.yaml 中的腿长还没有用于替换这些常量；因此不要把这里的
        // IK 输出直接当作真实舵机安装角。后续应让真实链结构与机械图纸一致。
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
    leg_calc::GaitConfig gait_config_{};
    leg_calc::GaitPhaseManager gait_phase_manager_;
    leg_calc::FootTrajectory foot_trajectory_;
    geometry_msgs::msg::Twist latest_task_cmd_vel_{};
    StaticLayoutConfig layout_config_{};
    leg_calc::SpiderFrameBundle frame_bundle_{};
    KDL::Chain demo_chain_;
    std::shared_ptr<leg_calc::LegKinematics> kinematics_;
    std::vector<leg_calc::ServoMapEntry> servo_map_{};
    std::string servo_map_path_;
    std::string leg_params_path_;
    rclcpp::Publisher<robot_interfaces::msg::Servo18>::SharedPtr servo_target_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr task_cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    mutable std::mutex task_cmd_mutex_;
    int control_period_ms_{20};
    double control_period_sec_{0.02};
    bool motion_active_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegCalcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
