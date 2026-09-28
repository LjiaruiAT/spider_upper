#include "leg_calc/command_watchdog.hpp"

namespace leg_calc {

CommandWatchdog::CommandWatchdog(double timeout_sec) : timeout_sec_(timeout_sec) {}

void CommandWatchdog::on_command(double now_sec, bool is_motion_command) {
    last_command_sec_ = now_sec;
    has_received_ = true;
    last_was_motion_ = is_motion_command;
}

bool CommandWatchdog::stale_motion(double now_sec) const {
    // timeout <= 0 视为关闭：直接不设防。
    if (timeout_sec_ <= 0.0) {
        return false;
    }
    // 从未收到过命令：这是启动瞬间的正常状态，不是"过期"。
    if (!has_received_) {
        return false;
    }
    // 最后一条本来就是零命令：它过期与否不改变任何行为，
    // 判 true 只会在正常站立时刷假警报。
    if (!last_was_motion_) {
        return false;
    }
    // 注意用 `>` 而不是 `>=`：恰好等于 timeout 的这一拍还不算过期，
    // 这样 timeout 就是"允许的最大间隔"，语义更直观、也更好测。
    return (now_sec - last_command_sec_) > timeout_sec_;
}

double CommandWatchdog::age_sec(double now_sec) const {
    if (!has_received_) {
        return 0.0;
    }
    const double age = now_sec - last_command_sec_;
    // 时钟回退（比如 NTP 校时、仿真时间跳变）时不给负数——
    // 否则 age > timeout 会莫名其妙地成立，机器人会无故停下。
    return age > 0.0 ? age : 0.0;
}

void CommandWatchdog::set_timeout_sec(double timeout_sec) {
    timeout_sec_ = timeout_sec;
}

void CommandWatchdog::reset() {
    last_command_sec_ = 0.0;
    has_received_ = false;
    last_was_motion_ = false;
}

}  // namespace leg_calc
