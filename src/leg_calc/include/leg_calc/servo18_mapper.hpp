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

    // 标定参数（决策 D3：标定放在上位机，下位机保持透传兜底）：
    //   servo_ddeg = direction * math_ddeg + offset_ddeg
    //
    // direction   ：只能是 +1 或 -1，吃掉左右镜像 / 舵机安装方向；
    // offset_ddeg ：数学角为 0 时舵机应有的读数，单位 0.1°。
    //
    // 默认 (direction=1, offset=0) 表示"不标定、原样透传"。
    int direction{1};
    int offset_ddeg{0};
};

// 关节结果到 Servo18 的“执行器映射层”，不是运动学求解器。
// 运动学输出每条腿的 [coxa, femur, tibia] 弧度；本类根据 YAML 把它们
// 重新排列到 18 个舵机通道，转换成 0.1 度，并应用每一路的标定参数。
// 输出是"真实舵机角"，不是抽象的数学角。
class Servo18Mapper {
public:
    static constexpr std::size_t kServoChannelCount = 18;
    // 舵机协议 / 机械的合法角度区间（单位 0.1°），与下位机 ClampAngleDdeg 一致。
    static constexpr int kServoMinDdeg = 0;
    static constexpr int kServoMaxDdeg = 1800;

    // 标定后的 18 路角度 + 映射诊断。
    // angle_ddeg 已夹取到 [kServoMinDdeg, kServoMaxDdeg]；
    // out_of_range_count 记录"被夹取过"的通道数，供上层报警用。
    struct AngleResult {
        std::array<int16_t, kServoChannelCount> angle_ddeg{};
        std::size_t out_of_range_count{0};
    };

    static std::vector<ServoMapEntry> load_map_from_yaml(const std::string& yaml_path);
    static AngleResult to_angle_ddeg(
        const SpiderJointTargets& targets,
        const std::vector<ServoMapEntry>& servo_map);
    static std::string to_debug_string(const std::array<int16_t, kServoChannelCount>& angle_ddeg);

private:
    static int radians_to_ddeg(double rad);
    static LegId parse_leg_id(const std::string& text);
    static JointId parse_joint_id(const std::string& text);
    static std::size_t joint_index(JointId joint_id);
    static std::string trim(const std::string& text);
};

}  // namespace leg_calc
