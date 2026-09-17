// common_types 里的纯类型与坐标变换测试
//
// 这一层没有算法，但它是整条链的坐标系约定：
//   body_T_leg / body_point_to_leg_point / leg_point_to_body_point
//   JointLimits
//
// 坐标系的东西一旦错了，后面所有数学都会"看起来正常但结果不对"，
// 而且极难靠日志排查，所以这里逐条钉死。

#include <gtest/gtest.h>

#include <cmath>

#include "leg_calc/common_types.hpp"

namespace {

using leg_calc::JointVector;
using leg_calc::LegId;
using leg_calc::LegMountPose;

constexpr double kPi = 3.14159265358979323846;

}  // namespace

// ---------------------------------------------------------------------------
// 身体坐标系 <-> 腿坐标系
//
// 约定：p_body = body_T_leg · p_leg，因此反向是 body_T_leg^{-1} · p_body。
// 这是"先在身体坐标系里规划足端、再转到每条腿自己的坐标系做 IK"的基础。
// ---------------------------------------------------------------------------
TEST(CommonTypesTest, LegMountPoseIsInvertible) {
    const auto mount = leg_calc::make_leg_mount_pose(LegId::LeftFront, Eigen::Vector3d(0.18, 0.12, 0.0));

    const Eigen::Vector3d body_point(0.20, 0.15, -0.25);
    const auto leg_point = leg_calc::body_point_to_leg_point(body_point, mount);
    const auto back_to_body = leg_calc::leg_point_to_body_point(leg_point, mount);

    EXPECT_TRUE(back_to_body.isApprox(body_point, 1e-12));
}

// 默认站姿下，身体系目标减去安装原点就是腿坐标系里的点
TEST(CommonTypesTest, NominalStandPoseBecomesStraightDownInLegFrame) {
    const auto mount = leg_calc::make_leg_mount_pose(LegId::LeftFront, Eigen::Vector3d(0.18, 0.12, 0.0));

    // 身体系目标与安装原点的 x/y 相同，只差高度
    const Eigen::Vector3d body_point(0.18, 0.12, -0.25);
    const auto leg_point = leg_calc::body_point_to_leg_point(body_point, mount);

    EXPECT_NEAR(leg_point.x(), 0.0, 1e-12);
    EXPECT_NEAR(leg_point.y(), 0.0, 1e-12);
    EXPECT_NEAR(leg_point.z(), -0.25, 1e-12)
        << "中性姿态下脚尖应在自己的腿坐标系里正下方";
}

// yaw 旋转要生效：绕 z 轴转 90° 后，原来的 +x 方向应变成 +y
TEST(CommonTypesTest, LegMountPoseAppliesYaw) {
    const auto mount =
        leg_calc::make_leg_mount_pose(LegId::LeftFront, Eigen::Vector3d::Zero(), kPi / 2.0);

    // 腿坐标系里的 (1, 0, 0) 应当被转到身体坐标系的 (0, 1, 0)
    const auto body_point = leg_calc::leg_point_to_body_point(Eigen::Vector3d(1.0, 0.0, 0.0), mount);

    EXPECT_NEAR(body_point.x(), 0.0, 1e-12);
    EXPECT_NEAR(body_point.y(), 1.0, 1e-12);
    EXPECT_NEAR(body_point.z(), 0.0, 1e-12);
}

// 六条腿的安装原点必须互不相同，否则"六足组织"就没有意义
TEST(CommonTypesTest, SixLegMountOriginsAreDistinct) {
    leg_calc::SpiderFrameBundle bundle;
    const double xs[6] = {0.18, 0.0, -0.18, 0.18, 0.0, -0.18};
    const double ys[6] = {0.12, 0.12, 0.12, -0.12, -0.12, -0.12};

    for (std::size_t i = 0; i < leg_calc::kLegCount; ++i) {
        const auto leg_id = static_cast<LegId>(i);
        bundle.leg_mounts[i] = leg_calc::make_leg_mount_pose(leg_id, Eigen::Vector3d(xs[i], ys[i], 0.0));
    }

    for (std::size_t i = 0; i < leg_calc::kLegCount; ++i) {
        for (std::size_t j = i + 1; j < leg_calc::kLegCount; ++j) {
            EXPECT_FALSE(bundle.leg_mounts[i].body_T_leg.translation().isApprox(
                bundle.leg_mounts[j].body_T_leg.translation(), 1e-12))
                << "第 " << i << " 和第 " << j << " 条腿的安装原点重合了";
        }
    }
}

// 左右腿的判据必须与 LegId 的定义一致
TEST(CommonTypesTest, LeftRightClassificationMatchesLegIds) {
    EXPECT_TRUE(leg_calc::is_left_leg(LegId::LeftFront));
    EXPECT_TRUE(leg_calc::is_left_leg(LegId::LeftMiddle));
    EXPECT_TRUE(leg_calc::is_left_leg(LegId::LeftRear));
    EXPECT_FALSE(leg_calc::is_right_leg(LegId::LeftFront));

    EXPECT_TRUE(leg_calc::is_right_leg(LegId::RightFront));
    EXPECT_TRUE(leg_calc::is_right_leg(LegId::RightMiddle));
    EXPECT_TRUE(leg_calc::is_right_leg(LegId::RightRear));

    // 左右互斥且完备
    for (const auto leg_id : leg_calc::kAllLegIds) {
        EXPECT_NE(leg_calc::is_left_leg(leg_id), leg_calc::is_right_leg(leg_id));
    }
}

// ---------------------------------------------------------------------------
// 关节限位
//
// 数值上收敛的解不一定机械上转得到，所以要能和"够不着"一样被拒绝。
// ---------------------------------------------------------------------------
TEST(JointLimitsTest, ContainsAcceptsValuesInsideTheBox) {
    leg_calc::JointLimits limits;
    limits.min = (JointVector() << -1.0, -1.0, -1.0).finished();
    limits.max = (JointVector() << 1.0, 1.0, 1.0).finished();

    EXPECT_TRUE(limits.contains((JointVector() << 0.0, 0.0, 0.0).finished()));
    EXPECT_TRUE(limits.contains((JointVector() << -1.0, 1.0, 0.0).finished())) << "边界算在内";
}

TEST(JointLimitsTest, ContainsRejectsAnySingleJointOutOfRange) {
    leg_calc::JointLimits limits;
    limits.min = (JointVector() << -1.0, -1.0, -1.0).finished();
    limits.max = (JointVector() << 1.0, 1.0, 1.0).finished();

    // 逐个关节单独越界，都要被判为不合格
    EXPECT_FALSE(limits.contains((JointVector() << -1.2, 0.0, 0.0).finished()));
    EXPECT_FALSE(limits.contains((JointVector() << 0.0, 1.2, 0.0).finished()));
    EXPECT_FALSE(limits.contains((JointVector() << 0.0, 0.0, -1.5).finished()));
}
