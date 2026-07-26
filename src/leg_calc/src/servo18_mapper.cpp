#include "leg_calc/servo18_mapper.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace leg_calc {

std::vector<ServoMapEntry> Servo18Mapper::load_map_from_yaml(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open servo map file: " + yaml_path);
    }

    std::vector<ServoMapEntry> entries;
    ServoMapEntry current_entry{};
    bool has_channel = false;
    bool has_leg = false;
    bool has_joint = false;

    auto flush_entry = [&]() {
        if (has_channel || has_leg || has_joint) {
            if (!(has_channel && has_leg && has_joint)) {
                throw std::runtime_error("Incomplete servo map entry in: " + yaml_path);
            }
            entries.push_back(current_entry);
            current_entry = ServoMapEntry{};
            has_channel = false;
            has_leg = false;
            has_joint = false;
        }
    };

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        if (trimmed == "servo_map:") {
            continue;
        }
        if (trimmed.rfind("- channel:", 0) == 0) {
            flush_entry();
            current_entry.channel = static_cast<std::size_t>(std::stoul(trim(trimmed.substr(10))));
            has_channel = true;
            continue;
        }
        if (trimmed.rfind("leg:", 0) == 0) {
            current_entry.leg_id = parse_leg_id(trim(trimmed.substr(4)));
            has_leg = true;
            continue;
        }
        if (trimmed.rfind("joint:", 0) == 0) {
            current_entry.joint_id = parse_joint_id(trim(trimmed.substr(6)));
            has_joint = true;
            continue;
        }
    }
    flush_entry();

    if (entries.size() != kServoChannelCount) {
        throw std::runtime_error("Servo map entry count is not 18 in: " + yaml_path);
    }

    return entries;
}

std::array<int16_t, Servo18Mapper::kServoChannelCount> Servo18Mapper::to_angle_ddeg(
    const SpiderJointTargets& targets,
    const std::vector<ServoMapEntry>& servo_map) {
    std::array<int16_t, kServoChannelCount> angle_ddeg{};

    for (const auto& entry : servo_map) {
        if (entry.channel >= kServoChannelCount) {
            throw std::runtime_error("Servo map channel out of range");
        }
        const auto leg_index = static_cast<std::size_t>(entry.leg_id);
        const auto joint_index_value = joint_index(entry.joint_id);
        angle_ddeg[entry.channel] = radians_to_ddeg(targets.legs[leg_index].joints(static_cast<int>(joint_index_value)));
    }

    return angle_ddeg;
}

std::string Servo18Mapper::to_debug_string(const std::array<int16_t, kServoChannelCount>& angle_ddeg) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < angle_ddeg.size(); ++i) {
        oss << angle_ddeg[i];
        if (i + 1 < angle_ddeg.size()) {
            oss << ", ";
        }
    }
    oss << "]";
    return oss.str();
}

int16_t Servo18Mapper::radians_to_ddeg(double rad) {
    constexpr double kRadToDeg = 180.0 / M_PI;
    return static_cast<int16_t>(std::lround(rad * kRadToDeg * 10.0));
}

LegId Servo18Mapper::parse_leg_id(const std::string& text) {
    if (text == "lf") {
        return LegId::LeftFront;
    }
    if (text == "lm") {
        return LegId::LeftMiddle;
    }
    if (text == "lr") {
        return LegId::LeftRear;
    }
    if (text == "rf") {
        return LegId::RightFront;
    }
    if (text == "rm") {
        return LegId::RightMiddle;
    }
    if (text == "rr") {
        return LegId::RightRear;
    }
    throw std::runtime_error("Unknown leg id in servo_map: " + text);
}

JointId Servo18Mapper::parse_joint_id(const std::string& text) {
    if (text == "coxa") {
        return JointId::Coxa;
    }
    if (text == "femur") {
        return JointId::Femur;
    }
    if (text == "tibia") {
        return JointId::Tibia;
    }
    throw std::runtime_error("Unknown joint id in servo_map: " + text);
}

std::size_t Servo18Mapper::joint_index(JointId joint_id) {
    return static_cast<std::size_t>(joint_id);
}

std::string Servo18Mapper::trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
}

}  // namespace leg_calc
