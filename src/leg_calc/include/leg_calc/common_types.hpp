#pragma once

#include <Eigen/Dense>
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

struct SpiderFootTargets {
    std::array<Eigen::Vector3d, kLegCount> feet{[]() {
        std::array<Eigen::Vector3d, kLegCount> result{};
        for (auto& foot : result) {
            foot = Eigen::Vector3d::Zero();
        }
        return result;
    }()};
};

}  // namespace leg_calc
