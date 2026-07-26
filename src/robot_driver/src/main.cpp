#include <array>
#include <iomanip>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/servo18.hpp"

// 这是 robot_driver 的第一版最小骨架：
// 先只负责订阅 spider_task 发来的舵机目标，
// 然后把消息打包成 42 字节协议帧，并通过一个“假发送”接口
// 模拟未来发往真实 USB CDC / 串口设备的流程。
class RobotDriverNode : public rclcpp::Node {
public:
    RobotDriverNode()
        : Node("robot_driver_node"), device_connected_(false), device_name_("/dev/ttyACM0") {
        subscription_ = this->create_subscription<robot_interfaces::msg::Servo18>(
            "/spider/servo_target",
            10,
            std::bind(&RobotDriverNode::target_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "robot_driver_node started, listening to /spider/servo_target");
        RCLCPP_INFO(this->get_logger(), "Driver placeholder ready. Current transport target = %s", device_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "No real USB CDC device is connected yet, so send_frame() is running in fake-send mode");
    }

private:
    static std::array<uint8_t, 42> pack_frame(const robot_interfaces::msg::Servo18 & msg) {
        std::array<uint8_t, 42> frame{};

        frame[0] = 0xAA;
        frame[1] = 0x55;
        frame[2] = 36;
        frame[3] = msg.seq;

        for (size_t i = 0; i < msg.angle_ddeg.size(); ++i) {
            const int16_t angle = msg.angle_ddeg[i];
            frame[4 + i * 2] = static_cast<uint8_t>(angle & 0xFF);
            frame[5 + i * 2] = static_cast<uint8_t>((angle >> 8) & 0xFF);
        }

        uint8_t checksum = 0;
        for (size_t i = 0; i < 40; ++i) {
            checksum ^= frame[i];
        }
        frame[40] = checksum;
        frame[41] = 0xBB;

        return frame;
    }

    static std::string frame_to_hex_string(const std::array<uint8_t, 42> & frame) {
        std::ostringstream oss;
        oss << std::hex << std::uppercase << std::setfill('0');

        for (size_t i = 0; i < frame.size(); ++i) {
            oss << std::setw(2) << static_cast<int>(frame[i]);
            if (i + 1 < frame.size()) {
                oss << ' ';
            }
        }

        return oss.str();
    }

    bool send_frame(const std::array<uint8_t, 42> & frame) {
        const auto frame_hex = frame_to_hex_string(frame);

        // 当前还没有真实 USB CDC / 串口硬件，
        // 所以这里先保留“发送层接口”，只做模拟发送。
        if (!device_connected_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "[fake-send] device %s is not connected yet, frame will only be printed",
                device_name_.c_str());
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[fake-send] frame: %s", frame_hex.c_str());
            return true;
        }

        // 以后如果接入真实设备，真实 open/write/send 逻辑就放在这里。
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[send] frame: %s", frame_hex.c_str());
        return true;
    }

    void target_callback(const robot_interfaces::msg::Servo18::SharedPtr msg) {
        std::ostringstream oss;
        oss << "Driver received seq=" << static_cast<int>(msg->seq) << ", angles=[";

        for (size_t i = 0; i < msg->angle_ddeg.size(); ++i) {
            oss << msg->angle_ddeg[i];
            if (i + 1 < msg->angle_ddeg.size()) {
                oss << ", ";
            }
        }

        oss << "]";

        const auto frame = pack_frame(*msg);
        const bool sent = send_frame(frame);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s", oss.str().c_str());
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Driver send result: %s (%s mode)",
            sent ? "success" : "failed",
            device_connected_ ? "real-device" : "fake-send");
    }

    bool device_connected_;
    std::string device_name_;
    rclcpp::Subscription<robot_interfaces::msg::Servo18>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
