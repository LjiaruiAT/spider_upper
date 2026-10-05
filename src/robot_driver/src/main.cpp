#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/servo18.hpp"
#include "serial_port.hpp"

// robot_driver：订阅 /spider/servo_target，把 18 路角度打包成 42 字节协议帧，
// 再通过串口送给下位机（STM32F407）。
//
// 运行参数（launch 加载 config/driver.yaml）：
//   device_name / baud_rate   真实串口设备与速率（fake_send=false 时使用）
//   fake_send                 true = 只打印协议帧、不碰设备；false = 真发串口
//   readback_enabled          回读开关（尚未实现，必须为 false）
//
// 依赖硬件的项**失败就明确报错**，不静默降级：
//   · fake_send=false 但打不开设备 → 启动失败；
//   · 运行中断线 → 自动尝试重连（1 秒节流）；
//   · readback_enabled=true → 启动失败（回读未实现）。
class RobotDriverNode : public rclcpp::Node {
public:
    RobotDriverNode()
        : Node("robot_driver_node") {
        declare_parameter<std::string>("device_name", "/dev/ttyACM0");
        declare_parameter<int>("baud_rate", 115200);
        declare_parameter<bool>("fake_send", true);
        declare_parameter<bool>("readback_enabled", false);

        device_name_ = this->get_parameter("device_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        fake_send_ = this->get_parameter("fake_send").as_bool();
        const bool readback_enabled = this->get_parameter("readback_enabled").as_bool();

        if (!fake_send_) {
            std::string reason;
            if (!serial_.open(device_name_, baud_rate_, &reason)) {
                throw std::runtime_error("真实串口打开失败：" + reason);
            }
            RCLCPP_INFO(
                this->get_logger(), "真实串口已打开：%s @ %d", device_name_.c_str(), baud_rate_);
        }
        if (readback_enabled) {
            throw std::runtime_error("舵机回读尚未实现（依赖硬件）；readback_enabled 只能为 false");
        }

        subscription_ = this->create_subscription<robot_interfaces::msg::Servo18>(
            "/spider/servo_target",
            10,
            std::bind(&RobotDriverNode::target_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "robot_driver_node started, listening to /spider/servo_target");
        RCLCPP_INFO(
            this->get_logger(),
            "%s；device=%s, baud=%d",
            fake_send_ ? "[fake-send] 只打印协议帧，不碰设备" : "[real-send] 真发串口",
            device_name_.c_str(),
            baud_rate_);
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
        if (fake_send_) {
            const auto frame_hex = frame_to_hex_string(frame);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000, "[fake-send] frame: %s",
                frame_hex.c_str());
            return true;
        }

        std::string reason;
        if (!serial_.write_all(frame.data(), frame.size(), &reason)) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000, "串口发送失败：%s", reason.c_str());
            if (serial_.try_reopen()) {
                RCLCPP_WARN(this->get_logger(), "串口已重连：%s", serial_.device().c_str());
            }
            return false;
        }
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
        if (!sent) {
            RCLCPP_ERROR(this->get_logger(), "发送失败：seq=%u", static_cast<unsigned>(msg->seq));
        }
    }

    bool fake_send_{true};
    int baud_rate_{115200};
    std::string device_name_;
    SerialPort serial_;
    rclcpp::Subscription<robot_interfaces::msg::Servo18>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
