#include "serial_port.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

// 把设备名与错误拼成一条可读信息（用于 ROS 日志）。
std::string errno_text(const std::string & what) {
    return what + ": " + std::strerror(errno);
}

}  // namespace

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::to_termios_speed(int baud_rate, unsigned int & out) {
    switch (baud_rate) {
        case 9600:   out = B9600;   return true;
        case 19200:  out = B19200;  return true;
        case 38400:  out = B38400;  return true;
        case 57600:  out = B57600;  return true;
        case 115200: out = B115200; return true;
        case 230400: out = B230400; return true;
        default:     return false;
    }
}

bool SerialPort::open(const std::string & device, int baud_rate, std::string * reason) {
    close();

    device_ = device;
    baud_rate_ = baud_rate;

    unsigned int speed = 0;
    if (!to_termios_speed(baud_rate, speed)) {
        if (reason != nullptr) {
            *reason = "不支持的波特率: " + std::to_string(baud_rate);
        }
        return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        if (reason != nullptr) {
            *reason = errno_text("打开 " + device + " 失败");
        }
        disconnected_ = true;
        return false;
    }

    struct termios tty {};
    if (::tcgetattr(fd_, &tty) != 0) {
        if (reason != nullptr) {
            *reason = errno_text("tcgetattr 失败");
        }
        close();
        disconnected_ = true;
        return false;
    }

    ::cfsetospeed(&tty, speed);
    ::cfsetispeed(&tty, speed);
    tty.c_cflag &= ~PARENB;                             // 无校验
    tty.c_cflag &= ~CSTOPB;                             // 1 停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;                                 // 8 数据位
    tty.c_cflag &= ~CRTSCTS;                            // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);   // raw
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;                                // 读超时 0.5s（本类只写，不读）

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        if (reason != nullptr) {
            *reason = errno_text("tcsetattr 失败");
        }
        close();
        disconnected_ = true;
        return false;
    }

    ::tcflush(fd_, TCIOFLUSH);
    disconnected_ = false;
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::write_all(const uint8_t * data, std::size_t size, std::string * reason) {
    if (fd_ < 0) {
        if (reason != nullptr) {
            *reason = "串口未打开（" + device_ + "）";
        }
        return false;
    }

    std::size_t written = 0;
    while (written < size) {
        const ssize_t n = ::write(fd_, data + written, size - written);
        if (n > 0) {
            written += static_cast<std::size_t>(n);
            continue;
        }
        if (reason != nullptr) {
            *reason = errno_text("写入失败");
        }
        disconnected_ = true;
        close();
        return false;
    }
    return true;
}

bool SerialPort::try_reopen() {
    if (!disconnected_) {
        return false;
    }

    const std::time_t now = std::time(nullptr);
    if (now == last_reopen_attempt_) {
        return false;   // 1 秒节流
    }
    last_reopen_attempt_ = now;

    std::string reason;
    if (!open(device_, baud_rate_, &reason)) {
        disconnected_ = true;
        return false;
    }
    return true;
}
