#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/servo18.hpp"

class ServoListenerNode : public rclcpp::Node {
public:
    ServoListenerNode()
        : Node("servo_listener_node") {
        // 订阅 spider_task_node 发布的 /spider/servo_target。
        subscription_ = this->create_subscription<robot_interfaces::msg::Servo18>(
            "/spider/servo_target",
            10,
            std::bind(&ServoListenerNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "servo_listener_node started, listening to /spider/servo_target");
    }

private:
    void topic_callback(const robot_interfaces::msg::Servo18::SharedPtr msg) {
        std::ostringstream oss;
        oss << "Received seq=" << static_cast<int>(msg->seq) << ", angles=[";

        for (size_t i = 0; i < msg->angle_ddeg.size(); ++i) {
            oss << msg->angle_ddeg[i];
            if (i + 1 < msg->angle_ddeg.size()) {
                oss << ", ";
            }
        }

        oss << "]";
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", oss.str().c_str());
    }

    rclcpp::Subscription<robot_interfaces::msg::Servo18>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
