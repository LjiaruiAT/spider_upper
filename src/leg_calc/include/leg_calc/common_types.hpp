#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace leg_calc {

constexpr std::size_t kLegJointDoF = 3;
constexpr std::size_t kLegCount = 6;

using JointVector = Eigen::Matrix<double, static_cast<int>(kLegJointDoF), 1>;
using JointMatrix = Eigen::Matrix<double, static_cast<int>(kLegJointDoF), static_cast<int>(kLegJointDoF)>;

struct JointState {
    JointVector position{JointVector::Zero()};
    JointVector velocity{JointVector::Zero()};
};

struct FootPose {
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
    Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct LegMountPose {
    LegId leg_id{LegId::LeftFront};
    Eigen::Isometry3d body_T_leg{Eigen::Isometry3d::Identity()};
};

struct SpiderFrameBundle {
    BodyFrame body_frame{};
    std::array<LegMountPose, kLegCount> leg_mounts{};
};

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

inline Eigen::Vector3d body_point_to_leg_point(const Eigen::Vector3d& body_point, const LegMountPose& mount) {
    return mount.body_T_leg.inverse() * body_point;
}

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
