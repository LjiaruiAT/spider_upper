// 命令看门狗测试
//
// 这个类守的是"机器人会不会自己一直走下去"，所以测试重点不在正常路径，
// 而在**不该报警时会不会乱报**——因为假警报的代价是正常站立时一直刷 WARN，
// 会训练出"看见这个 WARN 也不当回事"的习惯，等于把真警报也一起废掉。

#include <gtest/gtest.h>

#include "leg_calc/command_watchdog.hpp"

using leg_calc::CommandWatchdog;

// 从未收到命令时不该判定为过期：这是启动瞬间的正常状态。
TEST(CommandWatchdog, NeverReceivedIsNotStale) {
    CommandWatchdog wd(0.25);
    EXPECT_FALSE(wd.has_received_command());
    EXPECT_FALSE(wd.stale_motion(100.0));
    EXPECT_DOUBLE_EQ(wd.age_sec(100.0), 0.0);
}

// 收到运动命令、还没到超时 → 不过期。
TEST(CommandWatchdog, FreshMotionCommandIsNotStale) {
    CommandWatchdog wd(0.25);
    wd.on_command(10.0, true);
    EXPECT_TRUE(wd.has_received_command());
    EXPECT_TRUE(wd.last_was_motion());
    EXPECT_FALSE(wd.stale_motion(10.0));
    EXPECT_FALSE(wd.stale_motion(10.20));
    EXPECT_TRUE(wd.stale_motion(10.30));
}

// 恰好等于超时的这一拍不算过期（实现用 `>` 而不是 `>=`）。
// 边界值钉死，避免以后改成 `>=` 时无人察觉。
TEST(CommandWatchdog, ExactlyAtTimeoutIsNotStale) {
    CommandWatchdog wd(0.25);
    wd.on_command(5.0, true);
    EXPECT_FALSE(wd.stale_motion(5.25));
    EXPECT_TRUE(wd.stale_motion(5.2500001));
}

// 最关键的一条：**零命令过期不算过期**。
//
// spider_task 站立时只在进入 stand 的那一刻发一次零，之后长时间不发。
// 若这里判 true，正常站立会一直刷假警报。
TEST(CommandWatchdog, ExpiredZeroCommandIsNotStale) {
    CommandWatchdog wd(0.25);
    wd.on_command(1.0, false);
    EXPECT_FALSE(wd.stale_motion(999.0));
}

// 从运动切到零命令后，即使零命令也过期了，依然不算过期。
TEST(CommandWatchdog, AfterMotionThenZeroIsNotStale) {
    CommandWatchdog wd(0.25);
    wd.on_command(1.0, true);
    wd.on_command(1.1, false);
    EXPECT_FALSE(wd.stale_motion(500.0));
    EXPECT_FALSE(wd.last_was_motion());
}

// 过期后收到新命令要立刻恢复，不能有额外的冷却时间。
TEST(CommandWatchdog, NewCommandRecoversImmediately) {
    CommandWatchdog wd(0.25);
    wd.on_command(0.0, true);
    ASSERT_TRUE(wd.stale_motion(1.0));
    wd.on_command(1.0, true);
    EXPECT_FALSE(wd.stale_motion(1.0));
    EXPECT_FALSE(wd.stale_motion(1.2));
}

// 零命令之后又来运动命令，看门狗要重新开始计时。
TEST(CommandWatchdog, MotionAfterZeroRestartsTimer) {
    CommandWatchdog wd(0.25);
    wd.on_command(0.0, false);
    ASSERT_FALSE(wd.stale_motion(100.0));
    wd.on_command(100.0, true);
    EXPECT_FALSE(wd.stale_motion(100.1));
    EXPECT_TRUE(wd.stale_motion(100.3));
}

TEST(CommandWatchdog, AgeSecReportsElapsedTime) {
    CommandWatchdog wd(0.25);
    wd.on_command(2.0, true);
    EXPECT_DOUBLE_EQ(wd.age_sec(2.5), 0.5);
    EXPECT_DOUBLE_EQ(wd.age_sec(2.0), 0.0);
}

// 时钟回退不该产生负的 age，也不该因此判定过期。
// （NTP 校时、仿真时间跳变都会造成这种情况。）
TEST(CommandWatchdog, ClockGoingBackwardsDoesNotReportStale) {
    CommandWatchdog wd(0.25);
    wd.on_command(1000.0, true);
    EXPECT_DOUBLE_EQ(wd.age_sec(999.0), 0.0);
    EXPECT_FALSE(wd.stale_motion(999.0));
}

// timeout <= 0 = 关闭看门狗。这是"用 ROS 参数把它关掉"的语义，
// 必须能真正关掉，否则等于没提供开关。
TEST(CommandWatchdog, NonPositiveTimeoutDisablesWatchdog) {
    CommandWatchdog wd(0.0);
    wd.on_command(0.0, true);
    EXPECT_FALSE(wd.stale_motion(1e6));

    CommandWatchdog negative(-1.0);
    negative.on_command(0.0, true);
    EXPECT_FALSE(negative.stale_motion(1e6));
}

TEST(CommandWatchdog, SetTimeoutTakesEffect) {
    CommandWatchdog wd(0.25);
    wd.on_command(0.0, true);
    ASSERT_TRUE(wd.stale_motion(0.3));

    wd.set_timeout_sec(10.0);
    EXPECT_DOUBLE_EQ(wd.timeout_sec(), 10.0);
    EXPECT_FALSE(wd.stale_motion(0.3));
}

// reset 必须回到"从未收到过命令"，而不是"收到过一条很旧的命令"——
// 否则 reset 之后会立刻被判过期，起步瞬间就把运动掐掉。
TEST(CommandWatchdog, ResetReturnsToInitialState) {
    CommandWatchdog wd(0.25);
    wd.on_command(1.0, true);
    ASSERT_TRUE(wd.stale_motion(2.0));

    wd.reset();
    EXPECT_FALSE(wd.has_received_command());
    EXPECT_FALSE(wd.last_was_motion());
    EXPECT_FALSE(wd.stale_motion(2.0));
    EXPECT_FALSE(wd.stale_motion(1e6));
}

// 默认值必须是**开启保护**的状态，且超时与 spider_task 的 cmd_vel_timeout_sec
// 一致。防止"默认值被填成 0 导致保护静默失效"。
TEST(CommandWatchdog, DefaultTimeoutProtects) {
    CommandWatchdog wd;
    EXPECT_DOUBLE_EQ(wd.timeout_sec(), 0.25);
    wd.on_command(0.0, true);
    EXPECT_FALSE(wd.stale_motion(0.2));
    EXPECT_TRUE(wd.stale_motion(0.3));
}
