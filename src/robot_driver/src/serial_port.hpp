#ifndef ROBOT_DRIVER_SERIAL_PORT_HPP
#define ROBOT_DRIVER_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <string>

// 极简串口发送层（POSIX termios，8N1，raw 模式）。
// 只负责：打开设备 → 写字节。USB CDC（/dev/ttyACM*）同样适用。
class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort &) = delete;
    SerialPort & operator=(const SerialPort &) = delete;

    // 打开并配置设备；失败返回 false，reason 带错误文本（strerror）。
    bool open(const std::string & device, int baud_rate, std::string * reason = nullptr);

    void close();

    bool is_open() const { return fd_ >= 0; }

    // 写全部字节；失败返回 false 并标记断开。
    bool write_all(const uint8_t * data, std::size_t size, std::string * reason = nullptr);

    // 断开后尝试重连（1 秒节流，避免刷屏）。重连成功返回 true。
    bool try_reopen();

    const std::string & device() const { return device_; }

private:
    static bool to_termios_speed(int baud_rate, unsigned int & out);

    int fd_{-1};
    std::string device_;
    int baud_rate_{115200};
    bool disconnected_{false};
    std::time_t last_reopen_attempt_{0};
};

#endif  // ROBOT_DRIVER_SERIAL_PORT_HPP
