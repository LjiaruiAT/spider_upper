#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chain.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/servo18.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "leg_calc/command_watchdog.hpp"
#include "leg_calc/common_types.hpp"
#include "leg_calc/foot_trajectory.hpp"
#include "leg_calc/gait_phase_manager.hpp"
#include "leg_calc/leg_chain.hpp"
#include "leg_calc/leg_kinematics.hpp"
#include "leg_calc/leg_layout.hpp"
#include "leg_calc/odometry_integrator.hpp"
#include "leg_calc/servo18_mapper.hpp"
#include "leg_calc/velocity_smoother.hpp"

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

// 判断"命令是否超出能力上限"时的容差：速度命令和上限都是浮点数，
// 两者恰好相等时不应该被报成超限。
constexpr double kCommandEpsilon = 1e-6;

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

// 关节限位配置，单位：度（来自 YAML）。使用前会转成弧度。
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
        throw std::runtime_error("Failed to open leg params file: " + yaml_path);
    }

    JointLimitsConfig config;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        auto parse_deg_value = [&](const std::string& key, double& target) {
            if (trimmed.rfind(key, 0) == 0) {
                target = std::stod(trim(trimmed.substr(key.size())));
                return true;
            }
            return false;
        };

        if (trimmed == "leg_params:") {
            continue;
        }
        if (parse_deg_value("coxa_min_deg:", config.coxa_min_deg)) {
            continue;
        }
        if (parse_deg_value("coxa_max_deg:", config.coxa_max_deg)) {
            continue;
        }
        if (parse_deg_value("femur_min_deg:", config.femur_min_deg)) {
            continue;
        }
        if (parse_deg_value("femur_max_deg:", config.femur_max_deg)) {
            continue;
        }
        if (parse_deg_value("tibia_min_deg:", config.tibia_min_deg)) {
            continue;
        }
        if (parse_deg_value("tibia_max_deg:", config.tibia_max_deg)) {
            continue;
        }
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
          foot_trajectory_(gait_config_),
          velocity_smoother_(gait_config_) {
        RCLCPP_INFO(this->get_logger(), "leg_calc_node started");
        RCLCPP_INFO(this->get_logger(), "Current stage: gait phase -> foot trajectory -> body frame -> leg frame -> IK -> Servo18");

        declare_parameter<int>("control_period_ms", 20);
        control_period_ms_ = this->get_parameter("control_period_ms").as_int();
        if (control_period_ms_ <= 0) {
            control_period_ms_ = 20;
        }
        control_period_sec_ = static_cast<double>(control_period_ms_) / 1000.0;

        // 起步 / 停步时"运动强度"从 0 到 1（或反向）过渡所需的秒数。
        // 曲线用五次多项式生成，两端的速度和加速度都是 0，所以不会产生冲击。
        // 建议至少覆盖 2 个步态周期，否则腿还没走完一步就被加速/减速。
        declare_parameter<double>("motion_ramp_duration_sec", 1.0);
        motion_ramp_duration_sec_ = this->get_parameter("motion_ramp_duration_sec").as_double();
        if (motion_ramp_duration_sec_ <= 0.0) {
            motion_ramp_duration_sec_ = 1.0;
        }

        // 命令看门狗的超时。默认 0.25s 与 spider_task 的 cmd_vel_timeout_sec 一致，
        // 而 spider_task 的发布周期是 20ms——0.25s 相当于容忍连丢 12 拍，
        // 正常抖动绝不会触发。<= 0 表示关闭看门狗。
        //
        // 为什么阈值要和上游一致：两边管的是同一件事（"命令还新不新"），只是层级不同。
        // 如果 leg_calc 更宽松，上游超时发零之后本地还要再等一截才生效，
        // 那段时间里"两个保护都不生效"。
        declare_parameter<double>("task_cmd_timeout_sec", 0.25);
        command_watchdog_.set_timeout_sec(this->get_parameter("task_cmd_timeout_sec").as_double());

        task_cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spider/task_cmd_vel",
            10,
            std::bind(&LegCalcNode::task_cmd_vel_callback, this, std::placeholders::_1));

        servo_target_publisher_ = this->create_publisher<robot_interfaces::msg::Servo18>("/spider/servo_target", 10);

        // /joint_states：给 robot_state_publisher -> RViz 用。
        // 数据源就是 IK 解出的关节角，不需要任何额外计算——这个发布点是"免费"的，
        // 只是把已经算出来的东西说出来而已。
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // TF：发布 odom -> spider_base 这条边。
        // robot_state_publisher 发的是 spider_base -> 各连杆（来自 URDF），
        // 加上这条边，RViz 才能把机器人画在"地面"上而不是钉在原点。
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        const auto spider_share = ament_index_cpp::get_package_share_directory("spider");
        servo_map_path_ = spider_share + "/config/servo_map.yaml";
        leg_params_path_ = spider_share + "/config/leg_params.yaml";

        // 腿长来自配置，链在这里按实际尺寸构造——代码里不再有写死的杆长。
        layout_config_ = leg_calc::load_leg_layout_from_yaml(leg_params_path_);
        leg_chain_ = leg_calc::build_leg_chain(
            layout_config_.coxa_length_m,
            layout_config_.femur_length_m,
            layout_config_.tibia_length_m);
        kinematics_ = std::make_shared<leg_calc::LegKinematics>(leg_chain_);
        kinematics_->set_position_offset(Eigen::Vector3d(0.0, 0.0, 0.0));

        // 径向可达预检查**不启用**（保持 LegKinematics 的默认 (0, +inf)）。
        //
        // 原因：真实链带上关节限位之后，可达集合已经不是一个干净的球壳，
        // 手工算出的区间要么太松（不起作用）、要么太紧（误拒合法姿态）。
        // "解是否可信"统一交给 FK 回代误差 + 关节限位判断——这两道都是硬判据，
        // 不依赖任何关于链形状的假设。

        joint_limits_ = to_joint_limits(load_joint_limits_config(leg_params_path_));
        servo_map_ = leg_calc::Servo18Mapper::load_map_from_yaml(servo_map_path_);
        frame_bundle_ = leg_calc::build_frame_bundle(layout_config_);

        foot_trajectory_.update_config(gait_config_);
        gait_phase_manager_.update_config(gait_config_);

        // 把"参数是否自洽"显式打出来：步长上限 + 步态频率 + 支撑相占比三者一确定，
        // 能支持的最大速度也就确定了。命令超过它时机器人**不会更快**，
        // 只是命令与现实脱节——所以这个数字必须一开始就可见。
        RCLCPP_INFO(
            this->get_logger(),
            "Gait capability: pattern=%d, frequency=%.2f Hz, stance_duration=%.3f s, "
            "step_limit=%.1f mm -> max_speed=%.3f m/s, max_lateral=%.3f m/s, max_turn=%.3f rad/s",
            static_cast<int>(gait_config_.pattern),
            gait_config_.frequency_hz,
            leg_calc::stance_duration_s(gait_config_),
            gait_config_.step_length_m * 1000.0,
            leg_calc::max_forward_speed_mps(gait_config_),
            leg_calc::max_lateral_speed_mps(gait_config_),
            leg_calc::max_turn_rate_rps(gait_config_));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(control_period_ms_),
            std::bind(&LegCalcNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Loaded leg layout from %s", leg_params_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Loaded servo_map from %s", servo_map_path_.c_str());
        // 把腿几何和限位显式打出来——这是"配置到底有没有被真正用上"的唯一证据。
        // （以前这里打印可达半径，现在预检查关掉了，改为打印杆长与归位姿态：
        //   只要这几个数字不是图纸值，就说明配置没被读进去。）
        RCLCPP_INFO(
            this->get_logger(),
            "腿几何: coxa=%.1f femur=%.1f tibia=%.1f mm (总展长 %.1f mm), 归位(腿局部)=[%.1f, %.1f, %.1f] mm",
            layout_config_.coxa_length_m * 1000.0,
            layout_config_.femur_length_m * 1000.0,
            layout_config_.tibia_length_m * 1000.0,
            (layout_config_.coxa_length_m + layout_config_.femur_length_m + layout_config_.tibia_length_m) * 1000.0,
            layout_config_.home_local_m.x() * 1000.0,
            layout_config_.home_local_m.y() * 1000.0,
            layout_config_.home_local_m.z() * 1000.0);
        RCLCPP_INFO(
            this->get_logger(),
            "关节限位(deg): coxa[%.0f,%.0f] femur[%.0f,%.0f] tibia[%.0f,%.0f]",
            joint_limits_.min(0) * 180.0 / 3.14159265358979323846,
            joint_limits_.max(0) * 180.0 / 3.14159265358979323846,
            joint_limits_.min(1) * 180.0 / 3.14159265358979323846,
            joint_limits_.max(1) * 180.0 / 3.14159265358979323846,
            joint_limits_.min(2) * 180.0 / 3.14159265358979323846,
            joint_limits_.max(2) * 180.0 / 3.14159265358979323846);

        publish_servo_target("neutral", build_nominal_body_targets(), 0.0);
    }

private:
    leg_calc::BodyFootTargets build_nominal_body_targets() const {
        // 归位姿态定义在**腿局部坐标系**里（配置字段 home_local_mm），
        // 这里再用每条腿自己的安装位姿把它搬到身体坐标系。
        //
        // 为什么用腿局部坐标而不是身体坐标：
        //   六条腿的局部 x 轴已经按各自的 yaw 转到朝外，所以"足端在髋轴外侧
        //   多远、下方多深"对六条腿是同一个描述，一个向量就够。
        //   而且这样得到的站姿天然是六条腿向外撑开的——如果反过来在身体系里
        //   写"足端在安装点正下方"，腿就得先向外伸、再折回来，姿态既别扭、
        //   又更容易顶到限位。
        leg_calc::BodyFootTargets targets;

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            targets.feet[index] = leg_calc::leg_point_to_body_point(
                layout_config_.home_local_m, frame_bundle_.leg_mounts[index]);
        }

        return targets;
    }

    leg_calc::BodyFootTargets build_motion_body_targets(
        const leg_calc::BodyTwist& body_twist,
        double motion_scale) {
        // 先从站立时的名义足端位置开始，再由当前相位和身体速度生成运动目标。
        // 因此 IK 每个周期看到的是一个随时间变化的身体坐标系点。
        //
        // motion_scale 是"运动强度"：0 时六条腿全部等于站姿（起步/停步过渡），
        // 1 时是全速步态。它同时缩放位移、转角和抬腿高度，
        // 这样 stand 与 gait 的输出在 scale = 0 处完全重合。
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
                body_twist,
                motion_scale);
        }

        return targets;
    }

    void control_loop() {
        const auto latest_cmd = snapshot_latest_task_cmd_vel();

        // 命令新鲜度检查：命令来源（spider_task）可能崩溃 / 卡死 / 被 OOM 杀掉。
        // 那些情况下它内部的超时保护根本不会被执行，leg_calc 就会拿着最后一条
        // 非零命令一直走下去。所以执行端必须自己再判断一次（见 command_watchdog.hpp）。
        //
        // 关键：这里**只把命令当作零**，不去动 motion_scale / velocity_smoother_。
        // 命令变零之后就走进"正常停步路径"——motion_scale 用五次曲线把强度降下来。
        // 刻意不为紧急情况另写一条停步路径：那种路径平时不被走到，真出事时才发现
        // 它有问题就晚了，而且它永远得不到端到端验证。
        const bool cmd_expired = command_watchdog_.stale_motion(this->now().seconds());
        if (cmd_expired && !stale_warned_) {
            stale_warned_ = true;
            RCLCPP_WARN(
                this->get_logger(),
                "%.3f s 未收到 /spider/task_cmd_vel（超时阈值 %.3f s），按停步处理；"
                "通常是 spider_task 已退出或卡死",
                command_watchdog_.age_sec(this->now().seconds()),
                command_watchdog_.timeout_sec());
        } else if (!cmd_expired) {
            stale_warned_ = false;
        }

        const bool wants_motion = !cmd_expired && !is_zero_command(latest_cmd);

        // 运动强度过渡：起步时 0 -> 1，停步时 1 -> 0，曲线用五次多项式。
        //
        // 这里**不需要**任何"进入/退出运动"的边界检测：motion_scale = 0 时
        // 足端在所有相位下都严格等于站立姿态，gait 与 stand 的输出在这个点上
        // 完全重合，所以两者之间的切换本来就不会跳。
        const double ramp_direction = wants_motion ? 1.0 : -1.0;
        ramp_progress_ = std::clamp(
            ramp_progress_ + ramp_direction * control_period_sec_ / motion_ramp_duration_sec_,
            0.0,
            1.0);
        const double motion_scale = leg_calc::quintic_ease(ramp_progress_);

        // 里程计：用**乘过 motion_scale** 的速度积分。为什么必须乘 motion_scale，
        // 见 odometry_integrator.hpp——起步/停步过渡期间脚并没有真把机身推动那么多，
        // 直接用命令速度积分会让机身在该停的时候还在幽灵般地移动。
        //
        // 放在下面那条"彻底停下"的早退**之前**，是为了让它也照常广播 TF：
        // TF 一旦断掉，RViz 里 robot_state_publisher 发的整棵子树会一起消失。
        {
            leg_calc::BodyTwist scaled_twist = velocity_smoother_.current();
            scaled_twist.linear *= motion_scale;
            scaled_twist.angular *= motion_scale;
            odometry_.update(scaled_twist, control_period_sec_);
        }
        publish_odometry();

        // 已经彻底停下：回到标称站姿，并复位相位与速度平滑器，
        // 让下次起步总从"相位 0 + 零速度"开始。
        // 此时 motion_scale 已经是 0，步态输出本来就等于站姿，这个切换是无缝的。
        // ⚠ 这里**不要**再调 command_watchdog_.reset()。
        // 看门狗记录的是"上游最后一次说话是什么时候"。一旦把它清成"从未收到过命令"，
        // stale_motion() 会立刻变回 false，于是上面那条过期的非零命令又被当成有效，
        // 机器人会重新开始走——正好是这次改动要消灭的那个故障。
        if (!wants_motion && ramp_progress_ <= 0.0) {
            gait_phase_manager_.reset();
            velocity_smoother_.reset();
            publish_servo_target("stand", build_nominal_body_targets(), 0.0);
            return;
        }

        // 速度平滑只在"想运动"时推进，两种情形要分开看：
        //
        //   · 起步 / 运行中改速度 —— 平滑地跟踪目标速度。
        //     没有这一步，命令 0.2 → −0.2 会让步长在一个控制周期内从 +40mm
        //     跳到 −40mm，足端一步跨出 80mm。
        //
        //   · 停步 —— **冻结**在当前速度，减速交给 motion_scale。
        //     若这里也去跟踪零命令，步长会立刻归零、足端塌回标称站姿
        //     （实测水平跳变 20mm），而那时 motion_scale 还接近 1，来不及起作用。
        //
        // 平滑器冻结的值同时也是"停步期间应继续使用的最后一条有效速度"，
        // 所以不需要再单独缓存一份命令。
        if (wants_motion) {
            warn_if_command_exceeds_capability(latest_cmd);
            velocity_smoother_.update(to_body_twist(latest_cmd), control_period_sec_);
        }

        gait_phase_manager_.tick(control_period_sec_);
        publish_servo_target(
            wants_motion ? "gait" : "gait_stop",
            build_motion_body_targets(velocity_smoother_.current(), motion_scale),
            motion_scale);
    }

    // 命令速度超过当前步态参数所能支持的上限时，轨迹层会把步长夹住：
    // 机器人不会按该速度运动，只是命令与现实脱节。必须报出来，
    // 否则你会以为它真的在按 0.2 m/s 走。
    // 注意不能声明为 const：RCLCPP_*_THROTTLE 需要非 const 的 rclcpp::Clock。
    void warn_if_command_exceeds_capability(const geometry_msgs::msg::Twist& cmd) {
        const double max_vx = leg_calc::max_forward_speed_mps(gait_config_);
        const double max_vy = leg_calc::max_lateral_speed_mps(gait_config_);
        const double max_wz = leg_calc::max_turn_rate_rps(gait_config_);

        const bool within_capability =
            std::fabs(cmd.linear.x) <= max_vx + kCommandEpsilon &&
            std::fabs(cmd.linear.y) <= max_vy + kCommandEpsilon &&
            std::fabs(cmd.angular.z) <= max_wz + kCommandEpsilon;
        if (within_capability) {
            return;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "命令超出当前步态参数能力：cmd=(%.3f, %.3f, %.3f)，上限=(%.3f, %.3f, %.3f)。"
            "步长已被夹取，机器人不会按该速度运动；请提高步态频率或放宽步长上限。",
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,
            max_vx,
            max_vy,
            max_wz);
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
        //     -> ① 径向可达性预判（超出球壳直接拒绝，不必让求解器白迭代）
        //     -> ② IK 求 q
        //     -> ③ FK 回代，用误差判断这个解可不可信
        //     -> ④ 关节限位校验
        //     -> 不可信则保留上一条可信解
        //
        // 六条腿当前共用同一条 demo KDL chain；真正的机器人通常还需要根据
        // 左右侧镜像、腿座方向和每条腿实际尺寸建立正确的链。
        leg_calc::SpiderJointTargets spider_targets;

        // 诊断统计。solve_joint_targets() 每个控制周期都会被调用（默认 20ms），
        // 所以不能对每条腿各打一条 WARN，否则日志会被刷爆；这里先统计，循环结束后汇总一条。
        std::size_t unreachable_count = 0;
        std::size_t out_of_limit_count = 0;
        std::size_t ik_error_count = 0;
        std::size_t rejected_count = 0;
        std::size_t never_solved_count = 0;
        double max_error_norm = 0.0;

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            const auto& body_target = body_foot_targets.feet[index];
            const auto& mount = frame_bundle_.leg_mounts[index];
            // KDL 链的根坐标系是这条腿的局部坐标系，所以身体系目标必须先逆变换。
            const auto leg_target = leg_calc::body_point_to_leg_point(body_target, mount);
            const auto& last_trusted = last_trusted_joints_[index];

            // ① 径向可达性预判：够不着就不必调用求解器。
            if (!kinematics_->is_within_reach(leg_target)) {
                ++unreachable_count;
                ++rejected_count;
                spider_targets.legs[index].joints = last_trusted;
                continue;
            }

            // ② 数值 IK
            int ik_result = -1;
            const auto joint_solution = kinematics_->inverse_position(leg_target, &ik_result);
            // FK 回代不是第二次求解，而是检查 q 经正运动学后是否回到了目标点。
            const auto reconstructed_position = kinematics_->forward_position(joint_solution);

            // 回代误差在"腿坐标系"下比较，因为 IK 的目标点正是这个坐标系下的 leg_target。
            // LegKinematics 内部对 position_offset_ 的处理在 IK/FK 两边是对称的，
            // 所以这里直接用 reconstructed_position - leg_target，不需要再补偏移。
            const double error_norm = (reconstructed_position - leg_target).norm();
            max_error_norm = std::max(max_error_norm, error_norm);
            if (ik_result < 0) {
                ++ik_error_count;
            }

            // ③ 关节限位：数值上收敛不等于机械上转得到。
            const bool within_limits = joint_limits_.contains(joint_solution);
            if (!within_limits) {
                ++out_of_limit_count;
            }

            // ④ 判据是 FK 回代误差，不是 KDL 返回码。
            //    不可达时求解器不会说"无解"，而是返回最接近的位置——
            //    只看返回码会把"够不着"当成"解出来了"。
            const bool trustworthy = (error_norm <= kIkErrorToleranceM) && within_limits;

            if (trustworthy) {
                last_trusted_joints_[index] = joint_solution;
                has_trusted_joints_[index] = true;
                spider_targets.legs[index].joints = joint_solution;
            } else {
                // 拒绝不可信解、保留上一条：也就是"够不着就保持不动"。
                // 把解不出来的角度照发出去，在真机上就是舵机顶死或者乱动。
                ++rejected_count;
                if (!has_trusted_joints_[index]) {
                    // 自启动以来就没成功过：没有"上一条可信解"可用，
                    // 只能输出零位。这一定意味着配置有问题，必须单独指出。
                    ++never_solved_count;
                }
                spider_targets.legs[index].joints = last_trusted;
            }

            RCLCPP_DEBUG(
                this->get_logger(),
                "[%s] leg=%s body_target=[%.4f, %.4f, %.4f] ik=%d err=%.2fmm limits=%d accepted=%d joints=[%.4f, %.4f, %.4f]",
                tag.c_str(),
                leg_name_cstr(leg_id),
                body_target.x(),
                body_target.y(),
                body_target.z(),
                ik_result,
                error_norm * 1000.0,
                within_limits ? 1 : 0,
                trustworthy ? 1 : 0,
                joint_solution(0),
                joint_solution(1),
                joint_solution(2));
        }

        // 汇总：只要有一条腿被拒绝，就说明"机器人不会按指令动"，必须报出来。
        if (rejected_count > 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "[%s] 有 %zu/%zu 条腿的解被拒绝（保留上一条可信解）：够不着 %zu、超关节限位 %zu、IK 报错 %zu；"
                "其中 %zu 条自启动以来从未解出过可信解（正在输出零位角度，说明 leg_params.yaml 的站姿或关节限位配置有问题）；"
                "本轮最大 FK 回代误差=%.2f mm",
                tag.c_str(),
                rejected_count,
                leg_calc::kLegCount,
                unreachable_count,
                out_of_limit_count,
                ik_error_count,
                never_solved_count,
                max_error_norm * 1000.0);
        } else {
            RCLCPP_DEBUG(
                this->get_logger(),
                "[%s] 全部六条腿的解均可信，最大 FK 回代误差=%.4f mm",
                tag.c_str(),
                max_error_norm * 1000.0);
        }

        return spider_targets;
    }

    // 把六条腿的关节角发成 /joint_states，供 robot_state_publisher 算 TF、
    // RViz 画机器人。
    //
    // 关节角**直接就是 IK 的解，不需要任何换算**：URDF 里 coxa 绕 z、
    // femur/tibia 绕 y，与 leg_chain.cpp 的 RotZ / RotY 是同一套右手系约定。
    // 这一条如果错了，RViz 里的腿会朝反方向动——很难看出是符号问题，
    // 所以特意在这里写明白。
    //
    // 关节名走 leg_calc::joint_state_name()，它和 generate_urdf.py 是跨语言契约，
    // 名字对不上时腿不会动而且不报错（见 servo18_mapper.hpp 的说明）。
    // 发布 odom -> spider_base。
    //
    // 为什么必须有这条边：URDF 的根 link 是 spider_base，**按定义它永远在世界
    // 原点**。RViz 的 Fixed Frame 只能设成它，于是机身被钉死，你只能看到六条腿
    // 在原地划水——机器人其实在往前走，但这件事没有任何地方表达出来。
    //
    // 加上 odom 之后，把 Fixed Frame 改成 odom 就能看到机器人横穿地面，
    // 而**支撑足在世界系里站住不动**——那才是"走路"看起来应该有的样子。
    //
    // ⚠ 积分用的是命令速度，不是传感器数据。打滑或舵机跟不上时它就不准了，
    //    所以它只是**可视化辅助**，不能当真实里程计用（详见 odometry_integrator.hpp）。
    void publish_odometry() {
        const auto& odom_T_base = odometry_.odom_T_base();

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp = this->now();
        // 约定的方向：父 = odom（地面），子 = 机身。与 TF 一致。
        msg.header.frame_id = "odom";
        msg.child_frame_id = frame_bundle_.body_frame.frame_id;

        const Eigen::Vector3d translation = odom_T_base.translation();
        msg.transform.translation.x = translation.x();
        msg.transform.translation.y = translation.y();
        msg.transform.translation.z = translation.z();

        const Eigen::Quaterniond rotation(odom_T_base.linear());
        msg.transform.rotation.w = rotation.w();
        msg.transform.rotation.x = rotation.x();
        msg.transform.rotation.y = rotation.y();
        msg.transform.rotation.z = rotation.z();

        tf_broadcaster_->sendTransform(msg);
    }

    void publish_joint_states(const leg_calc::SpiderJointTargets& targets) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name.reserve(leg_calc::kLegCount * 3);
        msg.position.reserve(leg_calc::kLegCount * 3);

        for (const auto leg_id : leg_calc::kAllLegIds) {
            const auto index = leg_calc::leg_index(leg_id);
            for (const auto joint_id : {leg_calc::JointId::Coxa, leg_calc::JointId::Femur, leg_calc::JointId::Tibia}) {
                msg.name.push_back(leg_calc::joint_state_name(leg_id, joint_id));
                msg.position.push_back(targets.legs[index].joints(static_cast<int>(joint_id)));
            }
        }

        joint_state_publisher_->publish(msg);
    }

    void publish_servo_target(
        const std::string& tag,
        const leg_calc::BodyFootTargets& body_foot_targets,
        double motion_scale) {
        const auto spider_targets = solve_joint_targets(body_foot_targets, tag);
        // 关节角与舵机帧发的是同一批解：RViz 里看到的姿态就是真正发下去的姿态，
        // 不是另算的一套"显示用"数据。这样 RViz 才能当调试工具用。
        publish_joint_states(spider_targets);
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

        // 命令快照走加锁读取，不要直接碰 latest_task_cmd_vel_（它受 task_cmd_mutex_ 保护）。
        // 同时打印"收到的命令"和"实际采用的速度"：两者不同时才说明平滑器在起作用，
        // 只看其中一个都无法判断这件事。
        const auto cmd = snapshot_latest_task_cmd_vel();
        const auto& applied = velocity_smoother_.current();
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[%s] scale=%.3f, body_frame=%s, cmd=(%.3f, %.3f, %.3f), applied=(%.3f, %.3f, %.3f), cmd_age=%.3f s, Servo18=%s",
            tag.c_str(),
            motion_scale,
            frame_bundle_.body_frame.frame_id.c_str(),
            cmd.linear.x,
            cmd.linear.y,
            cmd.angular.z,
            applied.linear.x(),
            applied.linear.y(),
            applied.angular.z(),
            // 命令年龄：看门狗起作用时，这个数会一直涨到超过 task_cmd_timeout_sec。
            // 排查"为什么停了"时，这是最直接的一个数。
            command_watchdog_.age_sec(this->now().seconds()),
            leg_calc::Servo18Mapper::to_debug_string(servo_angles).c_str());
    }

    void task_cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(task_cmd_mutex_);
            latest_task_cmd_vel_ = *msg;
        }
        // 告诉看门狗"命令来了"，同时说明这条是不是运动命令。
        // spider_task 站立时只在进入 stand 的那一刻发一次零，之后长时间不发；
        // 若对零命令也判过期，正常站立会一直刷假警报。
        command_watchdog_.on_command(this->now().seconds(), !is_zero_command(*msg));

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received /spider/task_cmd_vel in leg_calc: vx=%.3f, vy=%.3f, wz=%.3f",
            msg->linear.x,
            msg->linear.y,
            msg->angular.z);
    }

    uint8_t sequence_;
    leg_calc::GaitConfig gait_config_{};
    leg_calc::GaitPhaseManager gait_phase_manager_;
    leg_calc::FootTrajectory foot_trajectory_;
    // 速度命令平滑器。它持有"当前速度"，必须是长期成员而不是每周期重建；
    // 同时它也是停步减速阶段"最后一条有效速度"的来源（停步时被冻结）。
    leg_calc::VelocitySmoother velocity_smoother_;
    // 命令看门狗：命令来源断掉时把命令当作零，从而走正常停步路径。
    leg_calc::CommandWatchdog command_watchdog_;
    // 超时告警只在状态翻转的那一拍报一次，避免每 20ms 刷一条。
    bool stale_warned_{false};
    // 按命令速度积分出的机身位姿，用来发 odom -> spider_base（见 odometry_integrator.hpp）。
    leg_calc::OdometryIntegrator odometry_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::Twist latest_task_cmd_vel_{};
    leg_calc::LegLayoutConfig layout_config_{};
    leg_calc::SpiderFrameBundle frame_bundle_{};
    leg_calc::JointLimits joint_limits_{};
    // 每条腿最近一次"可信"的关节解。遇到够不着 / 超关节限位 / 收敛失败时保留它，
    // 也就是"够不着就保持不动"，而不是把不可信的角度发出去。
    std::array<leg_calc::JointVector, leg_calc::kLegCount> last_trusted_joints_{};
    // 每条腿是否曾经解出过可信解。用来区分"暂时够不着（保留上一帧）"和
    // "自启动就解不出来（配置有问题，只能输出零位）"。
    std::array<bool, leg_calc::kLegCount> has_trusted_joints_{};
    KDL::Chain leg_chain_;
    std::shared_ptr<leg_calc::LegKinematics> kinematics_;
    std::vector<leg_calc::ServoMapEntry> servo_map_{};
    std::string servo_map_path_;
    std::string leg_params_path_;
    rclcpp::Publisher<robot_interfaces::msg::Servo18>::SharedPtr servo_target_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr task_cmd_vel_subscription_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    mutable std::mutex task_cmd_mutex_;
    int control_period_ms_{20};
    double control_period_sec_{0.02};
    double motion_ramp_duration_sec_{1.0};
    // 运动强度过渡的进度 [0, 1]。实际强度 = quintic_ease(ramp_progress_)。
    double ramp_progress_{0.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegCalcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
