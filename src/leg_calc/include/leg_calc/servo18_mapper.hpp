#pragma once

#include "leg_calc/common_types.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace leg_calc {

// 单腿关节的语义名称和 JointVector 下标保持一致。
// IK 输出的仍是弧度；这里的 mapper 只负责把它放到具体舵机通道。
enum class JointId {
    Coxa = 0,
    Femur = 1,
    Tibia = 2,
};

struct LegJointSet {
    JointVector joints{JointVector::Zero()};
};

struct SpiderJointTargets {
    std::array<LegJointSet, kLegCount> legs{};
};

struct ServoMapEntry {
    std::size_t channel{0};
    LegId leg_id{LegId::LeftFront};
    JointId joint_id{JointId::Coxa};
};

// 关节结果到 Servo18 的“执行器映射层”，不是运动学求解器。
// 运动学输出每条腿的 [coxa, femur, tibia] 弧度；本类根据 YAML 把它们
// 重新排列到 18 个舵机通道，并转换成协议使用的 0.1 度整数。
class Servo18Mapper {
public:
    static constexpr std::size_t kServoChannelCount = 18;

    static std::vector<ServoMapEntry> load_map_from_yaml(const std::string& yaml_path);
    static std::array<int16_t, kServoChannelCount> to_angle_ddeg(
        const SpiderJointTargets& targets,
        const std::vector<ServoMapEntry>& servo_map);
    static std::string to_debug_string(const std::array<int16_t, kServoChannelCount>& angle_ddeg);

private:
    static int16_t radians_to_ddeg(double rad);
    static LegId parse_leg_id(const std::string& text);
    static JointId parse_joint_id(const std::string& text);
    static std::size_t joint_index(JointId joint_id);
    static std::string trim(const std::string& text);
};

}  // namespace leg_calc
