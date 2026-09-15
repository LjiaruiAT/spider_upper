#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace leg_calc {

// 单腿当前按 3 自由度处理，关节顺序在整个 leg_calc 中必须保持一致：
//   q = [coxa, femur, tibia]^T
// 这里的角度单位是弧度（rad），位置/速度单位分别是米（m）和米/秒（m/s）。
// 后续 Servo18Mapper 才会把弧度转换为舵机协议使用的 0.1 度。
constexpr std::size_t kLegJointDoF = 3;
constexpr std::size_t kLegCount = 6;

// 单腿关节位置/速度向量：Eigen 下标 0/1/2 对应 coxa/femur/tibia。
using JointVector = Eigen::Matrix<double, static_cast<int>(kLegJointDoF), 1>;
// 当前只使用位置 Jacobian 的 3 行、3 列，因此暂时是固定的 3×3 矩阵。
using JointMatrix = Eigen::Matrix<double, static_cast<int>(kLegJointDoF), static_cast<int>(kLegJointDoF)>;

struct JointState {
    // position/velocity 均按 [coxa, femur, tibia] 排列；角度 rad，角速度 rad/s。
    JointVector position{JointVector::Zero()};
    JointVector velocity{JointVector::Zero()};
};

struct FootPose {
    // 足端位置的坐标系由调用上下文决定；leg_calc 的运动学接口使用腿局部坐标系，
    // FootTrajectory 生成的目标则使用身体坐标系。
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
};

struct FootState {
    FootPose pose{};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
};

enum class LegId {
    LeftFront = 0,
    LeftMiddle = 1,
    LeftRear = 2,
    RightFront = 3,
    RightMiddle = 4,
    RightRear = 5,
};

inline constexpr std::array<LegId, kLegCount> kAllLegIds{
    LegId::LeftFront,
    LegId::LeftMiddle,
    LegId::LeftRear,
    LegId::RightFront,
    LegId::RightMiddle,
    LegId::RightRear,
};

inline constexpr std::size_t leg_index(LegId leg_id) {
    return static_cast<std::size_t>(leg_id);
}

inline constexpr bool is_left_leg(LegId leg_id) {
    return leg_id == LegId::LeftFront || leg_id == LegId::LeftMiddle || leg_id == LegId::LeftRear;
}

inline constexpr bool is_right_leg(LegId leg_id) {
    return !is_left_leg(leg_id);
}

inline std::string leg_name(LegId leg_id) {
    switch (leg_id) {
    case LegId::LeftFront:
        return "lf";
    case LegId::LeftMiddle:
        return "lm";
    case LegId::LeftRear:
        return "lr";
    case LegId::RightFront:
        return "rf";
    case LegId::RightMiddle:
        return "rm";
    case LegId::RightRear:
        return "rr";
    default:
        throw std::runtime_error("Unknown leg id");
    }
}

struct BodyFrame {
    std::string frame_id{"spider_base"};
};

struct BodyTwist {
    // 当前控制链只使用平面运动：linear.x/y 为 m/s，angular.z 为 rad/s。
    // 它描述身体“想怎么动”，不是某一条腿的关节速度。
    Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct LegMountPose {
    LegId leg_id{LegId::LeftFront};

    // 齐次变换 body_T_leg：把“腿局部坐标系中的点”变换到“身体坐标系”。
    // 约定：p_body = body_T_leg * p_leg。
    // 当前主要使用平移；yaw 可以表达腿座相对身体的水平旋转。
    Eigen::Isometry3d body_T_leg{Eigen::Isometry3d::Identity()};
};

struct SpiderFrameBundle {
    BodyFrame body_frame{};
    std::array<LegMountPose, kLegCount> leg_mounts{};
};

// 创建一条腿的安装位姿。origin_in_body 是腿坐标系原点在身体坐标系中的位置，单位 m；
// yaw_rad 是绕身体/腿安装处 z 轴的水平旋转角，单位 rad。
inline LegMountPose make_leg_mount_pose(
    LegId leg_id,
    const Eigen::Vector3d& origin_in_body,
    double yaw_rad = 0.0) {
    LegMountPose mount;
    mount.leg_id = leg_id;
    mount.body_T_leg = Eigen::Isometry3d::Identity();
    mount.body_T_leg.translation() = origin_in_body;
    mount.body_T_leg.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return mount;
}

// 身体坐标系 -> 单腿局部坐标系。
// 运动学链的基座是腿局部坐标系，所以身体层产生的目标点在进入 IK 前必须先做：
//   p_leg = body_T_leg^{-1} * p_body
inline Eigen::Vector3d body_point_to_leg_point(const Eigen::Vector3d& body_point, const LegMountPose& mount) {
    return mount.body_T_leg.inverse() * body_point;
}

// 单腿局部坐标系 -> 身体坐标系。
// 这是上面变换的逆过程，常用于 FK 回代后的诊断或发布：
//   p_body = body_T_leg * p_leg
inline Eigen::Vector3d leg_point_to_body_point(const Eigen::Vector3d& leg_point, const LegMountPose& mount) {
    return mount.body_T_leg * leg_point;
}

struct SpiderFootTargets {
    std::array<Eigen::Vector3d, kLegCount> feet{[]() {
        std::array<Eigen::Vector3d, kLegCount> result{};
        for (auto& foot : result) {
            foot = Eigen::Vector3d::Zero();
        }
        return result;
    }()};
};

using BodyFootTargets = SpiderFootTargets;
using LegFootTargets = std::array<Eigen::Vector3d, kLegCount>;

}  // namespace leg_calc
