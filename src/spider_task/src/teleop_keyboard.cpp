// 键盘遥控节点：终端按键 → /spider/cmd_vel
//
// 数据流：
//   键盘 (WASD/QE/空格)
//     → geometry_msgs::Twist
//     → /spider/cmd_vel
//     → spider_task 的任务状态机（stand → move_request 自动切换）
//     → /spider/task_cmd_vel → leg_calc → 步态 → IK → 18 路舵机帧
//
// 为什么发 /spider/cmd_vel 而不是直接发 /spider/task_cmd_vel：
//   spider_task 里有显式任务状态机。往 /spider/cmd_vel 发**非零**命令时，
//   StandTask 的 process() 会自动返回 "move_request" 完成切换；命令过期
//   （cmd_vel_timeout_sec，默认 0.25s）或归零后，MoveRequestTask 自动切回
//   "stand" 并发零。等于免费得到一层"松手即停"的保护，不需要本节点操心。
//
// 关于"按住持续走"：
//   终端只能读到字符，**收不到按键释放事件**，所以无法做到字面意义的
//   "松开立刻停"。这里用标准做法（teleop_twist_keyboard 同思路）：
//     每收到一个键 → 更新命令 + 刷新时间戳
//     定时器里：距上次按键超过 kKeyTimeoutSec → 命令归零
//   "按住"是靠系统键盘重复（按住时约每 30ms 重发同一字符）持续刷新时间戳实现的。
//
//   副作用（无法消除，除非改用图形界面的键盘事件）：
//     · 按下瞬间有系统键盘重复延迟（默认约 0.25s）才动
//     · 松开后有约 0.3s 的"滑行"
//
// 速度：三个轴各自取"该轴能力上限的 75%"。上限从 gait_params.yaml 推算，
//       与 leg_calc 启动时打印的 max_speed / max_lateral / max_turn 同源。
//
// 按键：
//   W / S      前进 / 后退          (vx ±)
//   A / D      左移 / 右移          (vy ±，横移 strafe，机身朝向不变)
//   Q / E      左转 / 右转          (wz ±)
//   空格       急停（立即归零）
//   Ctrl-C     退出

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <termios.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "leg_calc/gait_config_loader.hpp"
#include "leg_calc/gait_types.hpp"

namespace {

// 速度上限的取用比例：最大能力的 75%。
constexpr double kSpeedRatio = 0.75;

// 发布周期：50 Hz。必须明显小于 spider_task 的 0.25s 看门狗，
// 否则正常"按住走动"期间也会被判超时。
constexpr double kPublishPeriodSec = 0.02;

// 多久没收到按键就把命令归零。
// 取值权衡：比系统键盘重复间隔（约 30ms）大得多，所以按住时不会误停；
// 又要尽量小，让"松手"响应快。0.3s 是与 spider_task 看门狗（0.25s）同量级的折中。
constexpr double kKeyTimeoutSec = 0.3;

// 终端原始模式的 RAII 守卫：构造时切换，析构时无条件恢复。
// 不恢复会让用户退出后终端不回显、不换行，属于"必须保证"的清理动作。
class RawTerminal {
public:
    RawTerminal() {
        if (::tcgetattr(STDIN_FILENO, &original_) != 0) {
            return;
        }
        struct termios raw = original_;
        // 关闭行缓冲与回显：否则要按回车才读到，且按键会回显到终端。
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        // VMIN=0 / VTIME=0：read() 非阻塞，没按键立刻返回 0。
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
            active_ = true;
        }
    }

    ~RawTerminal() { restore(); }

    RawTerminal(const RawTerminal &) = delete;
    RawTerminal & operator=(const RawTerminal &) = delete;

    void restore() {
        if (active_) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &original_);
            active_ = false;
        }
    }

    bool active() const { return active_; }

private:
    struct termios original_ {};
    bool active_{false};
};

class TeleopKeyboardNode : public rclcpp::Node {
public:
    TeleopKeyboardNode() : Node("teleop_keyboard_node") {
        // 步态模式会影响能力上限（不同步态的支撑相占比不同），所以允许覆盖。
        declare_parameter<std::string>("gait_pattern", "tripod");

        // 读取步态参数并推算三个轴的能力上限 —— 和 leg_calc 用同一个加载器、
        // 同一套换算函数，保证"遥控器以为的上限"和"机器人实际的上限"一致。
        const auto spider_share = ament_index_cpp::get_package_share_directory("spider");
        const std::string gait_yaml = spider_share + "/config/gait_params.yaml";
        leg_calc::GaitConfig config = leg_calc::load_gait_config_from_yaml(gait_yaml);
        config.pattern = leg_calc::parse_gait_pattern(
            this->get_parameter("gait_pattern").as_string());

        max_vx_ = leg_calc::max_forward_speed_mps(config) * kSpeedRatio;
        max_vy_ = leg_calc::max_lateral_speed_mps(config) * kSpeedRatio;
        max_wz_ = leg_calc::max_turn_rate_rps(config) * kSpeedRatio;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/spider/cmd_vel", 10);

        // 注意：RCLCPP_INFO 是 printf 风格格式化，"75%" 里的百分号必须写成 "%%"，
        // 否则 % 会被当成格式符，后面紧跟的中文字节会触发 -Wformat 警告且输出错乱。
        RCLCPP_INFO(this->get_logger(), "键盘遥控已就绪（%s 步态的 75%% 能力上限）",
                    leg_calc::gait_pattern_name(config.pattern));
        RCLCPP_INFO(this->get_logger(), "  W/S 前进后退 vx=%.3f m/s", max_vx_);
        RCLCPP_INFO(this->get_logger(), "  A/D 左移右移 vy=%.3f m/s", max_vy_);
        RCLCPP_INFO(this->get_logger(), "  Q/E 左转右转 wz=%.3f rad/s", max_wz_);
        RCLCPP_INFO(this->get_logger(), "  空格 急停；Ctrl-C 退出");
        RCLCPP_INFO(this->get_logger(),
                    "  提示：按住键才持续走；松开约 %.1fs 后停（终端收不到按键释放事件）",
                    kKeyTimeoutSec);

        last_key_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(kPublishPeriodSec),
            std::bind(&TeleopKeyboardNode::tick, this));
    }

    // 退出前把命令归零，避免节点没了但机器人还在按最后一条命令走。
    void publish_stop() {
        cmd_ = geometry_msgs::msg::Twist{};
        publisher_->publish(cmd_);
    }

private:
    void tick() {
        read_keys();

        // 超时未收到新键 → 归零。这样"松手"后不需要额外的释放事件。
        const double idle = (this->now() - last_key_time_).seconds();
        if (idle > kKeyTimeoutSec) {
            cmd_ = geometry_msgs::msg::Twist{};
        }

        publisher_->publish(cmd_);
    }

    // 非阻塞地把输入缓冲里的按键全部读掉（一次可能积压多个）。
    void read_keys() {
        char buffer[64];
        const ssize_t count = ::read(STDIN_FILENO, buffer, sizeof(buffer));
        if (count <= 0) {
            return;
        }
        for (ssize_t i = 0; i < count; ++i) {
            apply_key(buffer[i]);
        }
    }

    void apply_key(char key) {
        bool recognized = true;
        switch (key) {
        case 'w':
        case 'W':
            cmd_.linear.x = max_vx_;
            cmd_.linear.y = 0.0;
            break;
        case 's':
        case 'S':
            cmd_.linear.x = -max_vx_;
            cmd_.linear.y = 0.0;
            break;
        case 'a':
        case 'A':
            cmd_.linear.x = 0.0;
            cmd_.linear.y = max_vy_;
            break;
        case 'd':
        case 'D':
            cmd_.linear.x = 0.0;
            cmd_.linear.y = -max_vy_;
            break;
        case 'q':
        case 'Q':
            cmd_.angular.z = max_wz_;
            break;
        case 'e':
        case 'E':
            cmd_.angular.z = -max_wz_;
            break;
        case ' ':
            // 急停：立刻清空，并刷新时间戳，避免下一个 tick 又按旧命令发出去。
            cmd_ = geometry_msgs::msg::Twist{};
            last_key_time_ = this->now();
            return;
        default:
            recognized = false;
            break;
        }

        if (recognized) {
            last_key_time_ = this->now();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist cmd_;
    rclcpp::Time last_key_time_;

    // 三个轴各自的命令幅值（= 能力上限 × 75%）。
    double max_vx_{0.0};
    double max_vy_{0.0};
    double max_wz_{0.0};
};

}  // namespace

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    RawTerminal raw_terminal;
    if (!raw_terminal.active()) {
        // 终端没切成原始模式时，按键要按回车才生效、而且会回显 —— 能用但体验差，
        // 所以只警告不退出（在管道/重定向下运行仍然是合理的用法）。
        fprintf(stderr, "[teleop] 警告：无法切换终端原始模式（stdin 不是终端？），"
                        "需要按回车才触发按键。\n");
    }

    auto node = std::make_shared<TeleopKeyboardNode>();
    rclcpp::spin(node);

    // 收尾顺序很重要：先发零再恢复终端，保证"退出时机器人是停的"。
    node->publish_stop();
    raw_terminal.restore();
    rclcpp::shutdown();
    return 0;
}
