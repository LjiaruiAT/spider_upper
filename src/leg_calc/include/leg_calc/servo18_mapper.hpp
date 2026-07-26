#pragma once

#include "leg_calc/common_types.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace leg_calc {

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
