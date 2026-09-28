#pragma once

namespace leg_calc {

// 命令看门狗：判断"最后一条运动命令是不是已经过期了"。
//
// ---------------------------------------------------------------------------
// 为什么需要它
// ---------------------------------------------------------------------------
// leg_calc 的命令来源是 /spider/task_cmd_vel，由 spider_task 发布。
//
// 原来 leg_calc 只看"命令是不是零"：只要最后收到的是非零命令，就继续走。
// 这在 spider_task 正常工作时没问题——上游超时后它会主动发零。
//
// 但那份超时保护住在 **spider_task 自己体内**。它一旦
//   · 进程崩溃 / 被 OOM 杀掉 / 被 kill
//   · 调度线程死锁或卡住
//   · 订阅回调抛异常后不再进入
// 那段代码就根本不会被执行。此时 leg_calc 手上的最后一条非零命令永远"有效"，
// **机器人会一直走下去**——这是整个系统里唯一一处"无人值守也会持续动作"的缺口。
//
// 结论：超时保护必须在**执行端**再做一次。谁的脚在动，谁负责判断命令还新不新。
//
// ---------------------------------------------------------------------------
// 两个容易做错的地方
// ---------------------------------------------------------------------------
// ① **"从未收到命令" ≠ "命令过期了"**
//    启动瞬间还没有任何命令是正常状态，不该报警。所以两者要用不同的标志区分。
//
// ② **只有"运动命令"过期才值得处理**
//    spider_task 站立时只在进入 stand 的那一刻发一次零，之后长时间不发。
//    如果对零命令也判过期并报警，**正常站立会一直刷假警报**。
//    所以看门狗记录的是"最后一条命令是不是运动命令"，只对非零命令设防。
//
// 时间单位统一用秒（double），调用方传 `this->now().seconds()`。
class CommandWatchdog {
public:
    // timeout_sec <= 0 表示**关闭看门狗**（永远返回"不过期"）。
    // 这样可以通过 ROS 参数把它关掉，且行为可预测。
    explicit CommandWatchdog(double timeout_sec = 0.25);

    // 收到一条命令时调用。
    // is_motion_command：这条命令是否是"要动"（非零）。
    void on_command(double now_sec, bool is_motion_command);

    // 是否处于"运动命令已过期"状态——调用方据此把命令当作零处理。
    //
    // 只有同时满足以下三条才返回 true：
    //   · 看门狗没被关闭
    //   · 收到过命令
    //   · 最后一条命令是运动命令，且距今已超过 timeout_sec
    bool stale_motion(double now_sec) const;

    // 最后一条命令距今多久（秒）。从未收到过命令时返回 0。
    // 时钟回退（now < 上次）时返回 0，不会给出负数。
    double age_sec(double now_sec) const;

    bool has_received_command() const { return has_received_; }
    bool last_was_motion() const { return last_was_motion_; }

    double timeout_sec() const { return timeout_sec_; }
    void set_timeout_sec(double timeout_sec);

    // 清空全部状态，回到"从未收到过命令"。
    void reset();

private:
    double timeout_sec_{0.25};
    double last_command_sec_{0.0};
    bool has_received_{false};
    bool last_was_motion_{false};
};

}  // namespace leg_calc
