// 腿座布局测试
//
// 布局里最容易错的是**镜像**：右侧腿的 y 和 yaw 都要取负、x 不变。
// 这类错误不会崩、不会报错，只会让某几条腿朝反方向伸出去——
// 而且六条腿的 IK 都会"成功"，日志上看不出任何异常。
//
// 另一个重点是 yaw：它让每条腿的局部 x 轴指向身体外侧。
// 如果全部填 0，中腿就得靠 coxa 关节转 90° 才能伸到侧面，已经贴着限位了。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>

#include "leg_calc/leg_layout.hpp"

namespace {

using leg_calc::LegId;
using leg_calc::LegLayoutConfig;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

// 与 src/spider/config/leg_params.yaml 保持一致（改了要一起改）
LegLayoutConfig make_design_layout() {
    LegLayoutConfig config;
    config.coxa_length_m = 0.0603;
    config.femur_length_m = 0.0895;
    config.tibia_length_m = 0.154745;
    config.home_local_m = Eigen::Vector3d(0.1218, 0.0, -0.0914);

    config.front.x_m = 0.09652;
    config.front.y_m = 0.07513;
    config.front.yaw_rad = 45.0 * kDeg;

    config.middle.x_m = 0.0;
    config.middle.y_m = 0.1001;
    config.middle.yaw_rad = 90.0 * kDeg;

    config.rear.x_m = -0.09652;
    config.rear.y_m = 0.07513;
    config.rear.yaw_rad = 135.0 * kDeg;

    return config;
}

// 腿局部 x 轴在身体坐标系里的方向
Eigen::Vector3d local_x_in_body(const leg_calc::LegMountPose& mount) {
    return mount.body_T_leg.linear() * Eigen::Vector3d::UnitX();
}

Eigen::Vector3d mount_origin(const leg_calc::LegMountPose& mount) {
    return mount.body_T_leg.translation();
}

}  // namespace

// ---------------------------------------------------------------------------
// 镜像
// ---------------------------------------------------------------------------
TEST(LegLayoutTest, MirrorsRightLegsAcrossTheBodyAxis) {
    const auto bundle = leg_calc::build_frame_bundle(make_design_layout());

    const std::pair<LegId, LegId> pairs[] = {
        {LegId::LeftFront, LegId::RightFront},
        {LegId::LeftMiddle, LegId::RightMiddle},
        {LegId::LeftRear, LegId::RightRear},
    };

    for (const auto& [left_id, right_id] : pairs) {
        const auto& left = bundle.leg_mounts[leg_calc::leg_index(left_id)];
        const auto& right = bundle.leg_mounts[leg_calc::leg_index(right_id)];

        const Eigen::Vector3d left_origin = mount_origin(left);
        const Eigen::Vector3d right_origin = mount_origin(right);
        EXPECT_NEAR(right_origin.x(), left_origin.x(), 1e-12) << "镜像时 x 不变";
        EXPECT_NEAR(right_origin.y(), -left_origin.y(), 1e-12) << "镜像时 y 取负";

        const Eigen::Vector3d left_x = local_x_in_body(left);
        const Eigen::Vector3d right_x = local_x_in_body(right);
        EXPECT_NEAR(right_x.x(), left_x.x(), 1e-12);
        EXPECT_NEAR(right_x.y(), -left_x.y(), 1e-12) << "镜像时腿座朝向也要取负";
    }
}

TEST(LegLayoutTest, AllSixMountsAreDistinct) {
    const auto bundle = leg_calc::build_frame_bundle(make_design_layout());

    for (std::size_t i = 0; i < leg_calc::kLegCount; ++i) {
        for (std::size_t j = i + 1; j < leg_calc::kLegCount; ++j) {
            EXPECT_GT((mount_origin(bundle.leg_mounts[i]) - mount_origin(bundle.leg_mounts[j])).norm(), 1e-9)
                << "第 " << i << " 和第 " << j << " 条腿的安装点重合了";
        }
    }
}

// ---------------------------------------------------------------------------
// 腿座朝向
// ---------------------------------------------------------------------------
TEST(LegLayoutTest, EveryLegPointsAwayFromTheBodyCentre) {
    const auto bundle = leg_calc::build_frame_bundle(make_design_layout());

    for (const auto leg_id : leg_calc::kAllLegIds) {
        const auto& mount = bundle.leg_mounts[leg_calc::leg_index(leg_id)];
        // 局部 x 轴必须和"从身体中心指向安装点"的方向大致同向
        EXPECT_GT(local_x_in_body(mount).dot(mount_origin(mount)), 0.0)
            << leg_calc::leg_name(leg_id) << " 的腿座朝向了身体内侧（yaw 符号反了？）";
    }
}

TEST(LegLayoutTest, MiddleLegsPointSidewaysAndCornerLegsPointDiagonally) {
    const auto bundle = leg_calc::build_frame_bundle(make_design_layout());
    const double inv_sqrt2 = 1.0 / std::sqrt(2.0);

    struct Expectation {
        LegId leg;
        double x;
        double y;
    };
    const Expectation expected[] = {
        {LegId::LeftFront, inv_sqrt2, inv_sqrt2},     // +45°，朝左前
        {LegId::LeftMiddle, 0.0, 1.0},                // +90°，朝正左
        {LegId::LeftRear, -inv_sqrt2, inv_sqrt2},     // +135°，朝左后
        {LegId::RightFront, inv_sqrt2, -inv_sqrt2},   // −45°，朝右前
        {LegId::RightMiddle, 0.0, -1.0},              // −90°，朝正右
        {LegId::RightRear, -inv_sqrt2, -inv_sqrt2},   // −135°，朝右后
    };

    for (const auto& item : expected) {
        const Eigen::Vector3d dir = local_x_in_body(bundle.leg_mounts[leg_calc::leg_index(item.leg)]);
        EXPECT_NEAR(dir.x(), item.x, 1e-12) << leg_calc::leg_name(item.leg);
        EXPECT_NEAR(dir.y(), item.y, 1e-12) << leg_calc::leg_name(item.leg);
        EXPECT_NEAR(dir.z(), 0.0, 1e-12) << "腿座只绕竖直轴旋转，不应该有 z 分量";
    }
}

// ---------------------------------------------------------------------------
// 归位姿态：从腿局部坐标系映射到身体坐标系
//
// 这正是节点 build_nominal_body_targets() 做的事。手算对照能直接看出
// "站姿是不是向外撑开的"——如果映射写错，足端会跑到安装点正下方甚至内侧。
// ---------------------------------------------------------------------------
TEST(LegLayoutTest, HomeLocalMapsToTheExpectedBodyPoint) {
    const auto bundle = leg_calc::build_frame_bundle(make_design_layout());
    const Eigen::Vector3d& home_local = make_design_layout().home_local_m;

    // 左中腿：腿座在 (0, 0.1001)、朝正左（yaw 90°），
    // 所以足端应当落在 (0, 0.1001 + 0.1218, -0.0914)
    const auto lm = leg_calc::leg_point_to_body_point(
        home_local, bundle.leg_mounts[leg_calc::leg_index(LegId::LeftMiddle)]);
    EXPECT_NEAR(lm.x(), 0.0, 1e-9);
    EXPECT_NEAR(lm.y(), 0.1001 + 0.1218, 1e-9);
    EXPECT_NEAR(lm.z(), -0.0914, 1e-9);

    // 左前腿：腿座在 (0.09652, 0.07513)、朝左前 45°，
    // 足端外移量 0.1218 在 x/y 上各投影 0.1218·cos45°
    const double outward_xy = 0.1218 * std::cos(45.0 * kDeg);
    const auto lf = leg_calc::leg_point_to_body_point(
        home_local, bundle.leg_mounts[leg_calc::leg_index(LegId::LeftFront)]);
    EXPECT_NEAR(lf.x(), 0.09652 + outward_xy, 1e-9);
    EXPECT_NEAR(lf.y(), 0.07513 + outward_xy, 1e-9);
    EXPECT_NEAR(lf.z(), -0.0914, 1e-9);
}

// 六条腿的归位足端必须在**同一高度**——这是"站姿水平"的必要条件。
//
// 注意它们到身体中心的**水平距离并不相等**，而且这是设计本身的样子、不是 bug：
// 腿座不是圆形布置的（角腿在 96.52/75.13，中腿在 0/100.1），腿座朝向又是固定的
// 45° / 90° / 135° 而不是"各自对准圆心"，所以角腿的脚天然比中腿伸得更远。
// 下面把两组值都钉住，这样一旦 yaw 或镜像被改动会立刻暴露。
TEST(LegLayoutTest, HomePosesShareHeightAndKeepTheDesignedSpread) {
    const auto config = make_design_layout();
    const auto bundle = leg_calc::build_frame_bundle(config);

    double corner_horizontal = 0.0;
    double middle_horizontal = 0.0;

    for (const auto leg_id : leg_calc::kAllLegIds) {
        const Eigen::Vector3d foot =
            leg_calc::leg_point_to_body_point(config.home_local_m, bundle.leg_mounts[leg_calc::leg_index(leg_id)]);
        const double horizontal = std::sqrt(foot.x() * foot.x() + foot.y() * foot.y());

        EXPECT_NEAR(foot.z(), config.home_local_m.z(), 1e-9)
            << leg_calc::leg_name(leg_id) << " 的站姿高度和其他腿不一致";

        const bool is_middle =
            leg_id == LegId::LeftMiddle || leg_id == LegId::RightMiddle;
        double& reference = is_middle ? middle_horizontal : corner_horizontal;
        if (reference == 0.0) {
            reference = horizontal;
        }
        EXPECT_NEAR(horizontal, reference, 1e-9)
            << leg_calc::leg_name(leg_id) << " 的归位足端到中心的水平距离和同组其他腿不一致";
    }

    // 角腿应该比中腿伸得更远（腿座在外侧 + 朝向斜前方）
    EXPECT_GT(corner_horizontal, middle_horizontal)
        << "角腿的归位足端反而比中腿更靠近身体中心，说明 yaw 或安装位置有问题";
}

// ---------------------------------------------------------------------------
// YAML 加载
// ---------------------------------------------------------------------------
namespace {

const char* kTestYamlPath = "/tmp/leg_calc_test_leg_params.yaml";

void write_yaml(const std::string& content) {
    std::ofstream out(kTestYamlPath);
    out << content;
}

}  // namespace

TEST(LegLayoutYamlTest, LoadsMillimetresAndDegreesIntoMetresAndRadians) {
    write_yaml(
        "leg_params:\n"
        "  coxa_length_mm: 60.3\n"
        "  femur_length_mm: 89.5\n"
        "  tibia_length_mm: 154.745\n"
        "  home_local_mm: [121.8, 0.0, -91.4]\n"
        "  front_mount_x_mm: 96.52\n"
        "  front_mount_y_mm: 75.13\n"
        "  front_yaw_deg: 45.0\n"
        "  middle_mount_x_mm: 0.0\n"
        "  middle_mount_y_mm: 100.1\n"
        "  middle_yaw_deg: 90.0\n"
        "  rear_mount_x_mm: -96.52\n"
        "  rear_mount_y_mm: 75.13\n"
        "  rear_yaw_deg: 135.0\n");

    const auto config = leg_calc::load_leg_layout_from_yaml(kTestYamlPath);

    EXPECT_NEAR(config.coxa_length_m, 0.0603, 1e-12);
    EXPECT_NEAR(config.femur_length_m, 0.0895, 1e-12);
    EXPECT_NEAR(config.tibia_length_m, 0.154745, 1e-12);
    EXPECT_NEAR(config.home_local_m.x(), 0.1218, 1e-12);
    EXPECT_NEAR(config.home_local_m.z(), -0.0914, 1e-12);
    EXPECT_NEAR(config.front.yaw_rad, 45.0 * kDeg, 1e-12);
    EXPECT_NEAR(config.middle.yaw_rad, 90.0 * kDeg, 1e-12);
    EXPECT_NEAR(config.rear.yaw_rad, 135.0 * kDeg, 1e-12);
    EXPECT_NEAR(config.middle.y_m, 0.1001, 1e-12);

    std::remove(kTestYamlPath);
}

TEST(LegLayoutYamlTest, MissingHomeLocalIsRejected) {
    write_yaml(
        "leg_params:\n"
        "  coxa_length_mm: 60.3\n"
        "  femur_length_mm: 89.5\n"
        "  tibia_length_mm: 154.745\n");

    EXPECT_THROW(leg_calc::load_leg_layout_from_yaml(kTestYamlPath), std::runtime_error);
    std::remove(kTestYamlPath);
}

TEST(LegLayoutYamlTest, ZeroLinkLengthIsRejected) {
    // 杆长为 0 是最危险的情况：链是退化的，但 IK 仍会"成功"返回一些角度。
    // 必须在加载配置时就拦下来。
    write_yaml(
        "leg_params:\n"
        "  coxa_length_mm: 0.0\n"
        "  femur_length_mm: 89.5\n"
        "  tibia_length_mm: 154.745\n"
        "  home_local_mm: [121.8, 0.0, -91.4]\n");

    EXPECT_THROW(leg_calc::load_leg_layout_from_yaml(kTestYamlPath), std::runtime_error);
    std::remove(kTestYamlPath);
}

TEST(LegLayoutYamlTest, MalformedHomeLocalVectorIsRejected) {
    write_yaml(
        "leg_params:\n"
        "  coxa_length_mm: 60.3\n"
        "  femur_length_mm: 89.5\n"
        "  tibia_length_mm: 154.745\n"
        "  home_local_mm: [121.8, 0.0]\n");

    EXPECT_THROW(leg_calc::load_leg_layout_from_yaml(kTestYamlPath), std::runtime_error);
    std::remove(kTestYamlPath);
}
