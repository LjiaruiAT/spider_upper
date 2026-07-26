#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace {

bool is_zero_command(const geometry_msgs::msg::Twist& cmd) {
    constexpr double kEpsilon = 1e-6;
    return std::fabs(cmd.linear.x) < kEpsilon &&
           std::fabs(cmd.linear.y) < kEpsilon &&
           std::fabs(cmd.angular.z) < kEpsilon;
}

}  // namespace

// 这是一个最小的 ROS2 节点：
// 当前负责接收高层速度命令 /spider/cmd_vel，
// 并把任务层理解后的最小运动意图继续转发给 leg_calc。
// /spider/servo_target 的正式发布职责已经切到 leg_calc。
class SpiderTaskNode : public rclcpp::Node {
public:
    SpiderTaskNode()
        : Node("spider_task_node"), task_state_(TaskState::Stand) {
        task_cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/spider/task_cmd_vel", 10);

        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/spider/cmd_vel",
            10,
            std::bind(&SpiderTaskNode::cmd_vel_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "spider_task_node started, listening to /spider/cmd_vel and forwarding /spider/task_cmd_vel");
        RCLCPP_INFO(this->get_logger(), "spider_task_node no longer publishes /spider/servo_target directly; that role is handled by leg_calc");
        RCLCPP_INFO(this->get_logger(), "Initial task state = STAND");
    }

private:
    enum class TaskState {
        Stand,
        MoveRequest,
    };

    static const char* task_state_name(TaskState state) {
        switch (state) {
        case TaskState::Stand:
            return "STAND";
        case TaskState::MoveRequest:
            return "MOVE_REQUEST";
        default:
            return "UNKNOWN";
        }
    }

    void forward_task_intent() {
        geometry_msgs::msg::Twist intent = latest_cmd_vel_;
        if (task_state_ == TaskState::Stand) {
            intent = geometry_msgs::msg::Twist();
        }

        task_cmd_vel_publisher_->publish(intent);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Forwarding /spider/task_cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f, task_state=%s",
            intent.linear.x,
            intent.linear.y,
            intent.angular.z,
            task_state_name(task_state_));
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_cmd_vel_ = *msg;

        const TaskState new_state = is_zero_command(latest_cmd_vel_) ? TaskState::Stand : TaskState::MoveRequest;
        if (new_state != task_state_) {
            task_state_ = new_state;
            RCLCPP_INFO(
                this->get_logger(),
                "Task state changed to %s because latest /spider/cmd_vel = (%.3f, %.3f, %.3f)",
                task_state_name(task_state_),
                latest_cmd_vel_.linear.x,
                latest_cmd_vel_.linear.y,
                latest_cmd_vel_.angular.z);
        }

        forward_task_intent();

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Received /spider/cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f, task_state=%s",
            latest_cmd_vel_.linear.x,
            latest_cmd_vel_.linear.y,
            latest_cmd_vel_.angular.z,
            task_state_name(task_state_));
    }

    TaskState task_state_;
    geometry_msgs::msg::Twist latest_cmd_vel_{};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr task_cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpiderTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
