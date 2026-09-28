#include <array>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/servo18.hpp"

// 这是 robot_driver 的第一版最小骨架：
// 订阅 /spider/servo_target，把 18 路角度打包成 42 字节协议帧，
// 再通过 send_frame() 送去"发送层"（当前是 fake-send：只打印 HEX）。
//
// 运行参数从 ROS 参数来（launch 里加载 config/driver.yaml），不再写死在代码里：
//   device_name / baud_rate   真实串口用的设备与速率（尚未接入，仅记录）
//   fake_send                 true = 只打印协议帧、不碰设备（当前唯一支持的模式）
//   readback_enabled          回读开关（尚未实现，必须为 false）
//
// fake_send=false 或 readback_enabled=true 会**启动失败**：这两项功能都依赖硬件、
// 还没实现，静默降级会让人以为"正在真发送 / 正在回读"——那是更危险的状态。
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
            throw std::runtime_error(
                "真实串口发送尚未实现（依赖硬件，见 工程现状总结.md 6.6）；"
                "当前只支持 fake_send=true（仅打印协议帧，不碰设备）");
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
            "[fake-send] 只打印协议帧，不碰设备；device=%s, baud=%d（接入真实串口后才会用到）",
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
        // 当前只有 fake-send：只打印 HEX，不碰设备。
        // 接入真实串口的 open / configure / write 逻辑将来写在这里的分支之后——
        // 到那时 device_name_ / baud_rate_ 才会真正被用到（构造时已保证
        // fake_send=false 会直接启动失败，所以到这里必然是打印分支）。
        const auto frame_hex = frame_to_hex_string(frame);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "[fake-send] frame: %s", frame_hex.c_str());
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
    rclcpp::Subscription<robot_interfaces::msg::Servo18>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
