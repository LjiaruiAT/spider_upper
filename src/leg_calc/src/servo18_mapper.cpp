#include "leg_calc/servo18_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace leg_calc {

std::vector<ServoMapEntry> Servo18Mapper::load_map_from_yaml(const std::string& yaml_path) {
    // 这里把 YAML 中的“物理关节 -> 舵机通道”关系读入内存。
    // 映射层不改变关节含义，只决定哪个腿的哪个关节写入哪个 Servo18 下标。
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
        // 标定字段是**可选**的：不写就沿用 ServoMapEntry 的默认值 (direction=1, offset=0)，
        // 也就是"不标定、原样透传"。这样旧版 YAML 仍然能被读进来。
        if (trimmed.rfind("direction:", 0) == 0) {
            const int direction = std::stoi(trim(trimmed.substr(10)));
            if (direction != 1 && direction != -1) {
                throw std::runtime_error("servo_map 'direction' must be +1 or -1 in: " + yaml_path);
            }
            current_entry.direction = direction;
            continue;
        }
        if (trimmed.rfind("offset_ddeg:", 0) == 0) {
            current_entry.offset_ddeg = std::stoi(trim(trimmed.substr(12)));
            continue;
        }
    }
    flush_entry();

    if (entries.size() != kServoChannelCount) {
        throw std::runtime_error("Servo map entry count is not 18 in: " + yaml_path);
    }

    return entries;
}

Servo18Mapper::AngleResult Servo18Mapper::to_angle_ddeg(
    const SpiderJointTargets& targets,
    const std::vector<ServoMapEntry>& servo_map) {
    // 这里是"数学关节角 -> 真实舵机角"的完整映射，分三步：
    //   1) 单位转换：rad -> 0.1°（ddeg）
    //   2) 标定：servo = direction * math + offset
    //   3) 夹取到 [kServoMinDdeg, kServoMaxDdeg]，并统计被夹取的通道数
    //
    // 第 2 步是按 工程现状总结.md 决策 D3 新增的：标定放在上位机，因此
    // /spider/servo_target 里的 angle_ddeg 就是"真实舵机角"，下位机不需要
    // 再做同样换算，只保留硬夹取兜底。
    // 默认 (direction=1, offset=0) 时第 2 步退化为恒等，输出与旧版本完全一致。
    AngleResult result;

    for (const auto& entry : servo_map) {
        if (entry.channel >= kServoChannelCount) {
            throw std::runtime_error("Servo map channel out of range");
        }
        // 先用 leg_id 找到六腿数组，再用 joint_id 找到 [coxa,femur,tibia] 的列，
        // 最后写入 YAML 指定的 Servo18 channel。
        const auto leg_index = static_cast<std::size_t>(entry.leg_id);
        const auto joint_index_value = joint_index(entry.joint_id);
        const double math_rad = targets.legs[leg_index].joints(static_cast<int>(joint_index_value));

        // 全程用 int 计算：如果中间结果先落到 int16_t，越界值会被截断，
        // 反而看不出"这个角度根本不在合法区间"。
        const int math_ddeg = radians_to_ddeg(math_rad);
        const int servo_ddeg = entry.direction * math_ddeg + entry.offset_ddeg;

        if (servo_ddeg < kServoMinDdeg || servo_ddeg > kServoMaxDdeg) {
            ++result.out_of_range_count;
        }
        result.angle_ddeg[entry.channel] =
            static_cast<int16_t>(std::clamp(servo_ddeg, kServoMinDdeg, kServoMaxDdeg));
    }

    return result;
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

int Servo18Mapper::radians_to_ddeg(double rad) {
    // 协议单位是 deci-degree（十分之一度），而运动学统一使用弧度。
    // 返回 int 而不是 int16_t：极端角度下转成 int16_t 会得到实现相关的值，
    // 等于把"角度明显越界"这个信息丢掉；越界判断交给调用方做。
    constexpr double kRadToDeg = 180.0 / M_PI;
    return static_cast<int>(std::lround(rad * kRadToDeg * 10.0));
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
