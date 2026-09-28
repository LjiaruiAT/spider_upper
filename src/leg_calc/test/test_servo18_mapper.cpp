// Servo18Mapper 单元测试
//
// 这一层是"数学关节角 -> 真实舵机角"的翻译层，也是标定参数（offset /
// direction）唯一被应用的地方。它出错的后果很隐蔽：不会崩、不会报错，
// 只是所有舵机都偏了几个度，或者某几条腿整体反向。
//
// 本文件锁定四件事：
//   1. 默认标定 (direction=1, offset=0) 必须是纯单位换算，行为与未引入
//      标定前完全一致（回归安全）
//   2. direction=-1 取反、offset_ddeg 平移，且顺序正确
//   3. 越界的角度被夹到 [0, 1800]，并且**要被计数**，供上层报警
//   4. YAML 里的标定字段是可选的，缺省时退回恒等（向后兼容）

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "leg_calc/servo18_mapper.hpp"

namespace {

using leg_calc::joint_state_name;
using leg_calc::JointId;
using leg_calc::JointVector;
using leg_calc::kAllLegIds;
using leg_calc::LegId;
using leg_calc::Servo18Mapper;
using leg_calc::ServoMapEntry;
using leg_calc::SpiderJointTargets;

constexpr std::size_t kChannelCount = Servo18Mapper::kServoChannelCount;

// 构造一个"直通"映射：channel 0~2 -> LF 的 coxa/femur/tibia，
// channel 3~5 -> LM，依此类推，全部 direction=1 / offset=0。
std::vector<ServoMapEntry> make_identity_map() {
    std::vector<ServoMapEntry> map;
    map.reserve(kChannelCount);
    for (std::size_t channel = 0; channel < kChannelCount; ++channel) {
        ServoMapEntry entry;
        entry.channel = channel;
        entry.leg_id = static_cast<LegId>(channel / leg_calc::kLegJointDoF);
        entry.joint_id = static_cast<JointId>(channel % leg_calc::kLegJointDoF);
        map.push_back(entry);
    }
    return map;
}

// 生成 18 条（或指定条数）的 servo_map YAML。
// with_calibration=false 时故意不写 direction / offset_ddeg 两行，
// 用来验证解析器的向后兼容性。
std::string build_yaml(int entry_count, bool with_calibration, int first_direction = 1, int first_offset = 0) {
    static const char* kLegs[] = {"lf", "lm", "lr", "rf", "rm", "rr"};
    static const char* kJoints[] = {"coxa", "femur", "tibia"};

    std::ostringstream out;
    out << "servo_map:\n";
    for (int channel = 0; channel < entry_count; ++channel) {
        out << "  - channel: " << channel << "\n";
        out << "    leg: " << kLegs[channel / 3] << "\n";
        out << "    joint: " << kJoints[channel % 3] << "\n";
        if (with_calibration) {
            out << "    direction: " << (channel == 0 ? first_direction : 1) << "\n";
            out << "    offset_ddeg: " << (channel == 0 ? first_offset : 0) << "\n";
        }
    }
    return out.str();
}

std::string write_temp_yaml(const std::string& content, const std::string& name) {
    const std::string path = ::testing::TempDir() + name;
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

}  // namespace

// ---------------------------------------------------------------------------
// 默认标定 = 纯单位换算（回归安全）
//
// direction=1 / offset=0 时，输出必须和"没有标定功能"时完全一致：
//   angle_ddeg = round(rad * 180/pi * 10)
// 这条断言保证引入标定通路没有悄悄改变原有行为。
// ---------------------------------------------------------------------------
TEST(Servo18MapperTest, DefaultCalibrationIsPureUnitConversion) {
    SpiderJointTargets targets;
    // 0.10996 rad ≈ 6.3° -> 63 ddeg
    // 1.20079 rad ≈ 68.8° -> 688 ddeg
    targets.legs[leg_calc::leg_index(LegId::LeftFront)].joints =
        (JointVector() << 0.0, 0.10996, 1.20079).finished();

    const auto result = Servo18Mapper::to_angle_ddeg(targets, make_identity_map());

    EXPECT_EQ(result.angle_ddeg[0], 0);
    EXPECT_EQ(result.angle_ddeg[1], 63);
    EXPECT_EQ(result.angle_ddeg[2], 688);
    EXPECT_EQ(result.out_of_range_count, 0u);

    // 其余五条腿没被赋值，应保持 0
    for (std::size_t channel = 3; channel < kChannelCount; ++channel) {
        EXPECT_EQ(result.angle_ddeg[channel], 0);
    }
}

// ---------------------------------------------------------------------------
// 标定：direction 先取反，offset 后平移
// ---------------------------------------------------------------------------
TEST(Servo18MapperTest, NegativeDirectionFlipsSign) {
    auto map = make_identity_map();
    map[1].direction = -1;  // LF femur 反向

    SpiderJointTargets targets;
    targets.legs[leg_calc::leg_index(LegId::LeftFront)].joints =
        (JointVector() << 0.0, 0.10996, 0.0).finished();

    const auto result = Servo18Mapper::to_angle_ddeg(targets, map);

    // +63 被取反成 -63，超出 [0, 1800] 所以被夹到 0，并且被计数
    EXPECT_EQ(result.angle_ddeg[1], 0);
    EXPECT_EQ(result.out_of_range_count, 1u);
}

TEST(Servo18MapperTest, OffsetShiftsTheCalibratedAngle) {
    auto map = make_identity_map();
    map[1].offset_ddeg = 900;  // 数学角为 0 时舵机应读到 90.0°

    SpiderJointTargets targets;
    targets.legs[leg_calc::leg_index(LegId::LeftFront)].joints =
        (JointVector() << 0.0, 0.10996, 0.0).finished();

    const auto result = Servo18Mapper::to_angle_ddeg(targets, map);

    EXPECT_EQ(result.angle_ddeg[1], 963);  // 63 + 900
    EXPECT_EQ(result.out_of_range_count, 0u);
}

// ---------------------------------------------------------------------------
// 越界：夹取 + 计数
//
// 只夹取不报警是危险的——问题会被静默吞掉。所以这里同时验证两件事。
// ---------------------------------------------------------------------------
TEST(Servo18MapperTest, OutOfRangeIsClampedAndCounted) {
    SpiderJointTargets targets;
    // -0.5 rad ≈ -28.6° -> -286 ddeg，远低于合法下限
    targets.legs[leg_calc::leg_index(LegId::LeftFront)].joints =
        (JointVector() << 0.0, -0.5, 0.0).finished();

    const auto result = Servo18Mapper::to_angle_ddeg(targets, make_identity_map());

    EXPECT_EQ(result.angle_ddeg[1], 0) << "越界值应被夹到合法区间";
    EXPECT_EQ(result.out_of_range_count, 1u) << "越界必须被计数，供上层报警";
}

TEST(Servo18MapperTest, UpperBoundIsClampedToo) {
    SpiderJointTargets targets;
    // 5 rad ≈ 286° -> 2865 ddeg，超过 1800
    targets.legs[leg_calc::leg_index(LegId::LeftFront)].joints =
        (JointVector() << 0.0, 0.0, 5.0).finished();

    const auto result = Servo18Mapper::to_angle_ddeg(targets, make_identity_map());

    EXPECT_EQ(result.angle_ddeg[2], Servo18Mapper::kServoMaxDdeg);
    EXPECT_EQ(result.out_of_range_count, 1u);
}

TEST(Servo18MapperTest, RejectsChannelIndexOutOfRange) {
    auto map = make_identity_map();
    map[0].channel = 99;

    SpiderJointTargets targets;
    EXPECT_THROW(Servo18Mapper::to_angle_ddeg(targets, map), std::runtime_error);
}

// ---------------------------------------------------------------------------
// 通道重排
//
// 输出的第 N 个数由 YAML 里 channel=N 那条决定，而不是由关节顺序决定。
// 这是"上位机改配置就能换接线"的基础。
// ---------------------------------------------------------------------------
TEST(Servo18MapperTest, ChannelOrderFollowsTheMapNotTheJointOrder) {
    auto map = make_identity_map();
    // 让 channel 0 去读右后腿的 tibia
    map[0].leg_id = LegId::RightRear;
    map[0].joint_id = JointId::Tibia;

    SpiderJointTargets targets;
    targets.legs[leg_calc::leg_index(LegId::LeftFront)].joints =
        (JointVector() << 0.0, 0.0, 2.0).finished();  // 不该被读到
    targets.legs[leg_calc::leg_index(LegId::RightRear)].joints =
        (JointVector() << 0.0, 0.0, 1.0).finished();

    const auto result = Servo18Mapper::to_angle_ddeg(targets, map);

    // 1 rad ≈ 57.296° -> 573 ddeg，来自 RR 的 tibia 而不是 LF 的 tibia
    EXPECT_EQ(result.angle_ddeg[0], 573);
}

// ---------------------------------------------------------------------------
// YAML 解析
// ---------------------------------------------------------------------------
TEST(Servo18MapperTest, YamlParsingReadsCalibrationFields) {
    const auto path = write_temp_yaml(
        build_yaml(static_cast<int>(kChannelCount), true, -1, 900),
        "servo_map_with_calibration.yaml");

    const auto map = Servo18Mapper::load_map_from_yaml(path);

    ASSERT_EQ(map.size(), kChannelCount);
    EXPECT_EQ(map[0].direction, -1);
    EXPECT_EQ(map[0].offset_ddeg, 900);
    EXPECT_EQ(map[1].direction, 1);
    EXPECT_EQ(map[1].offset_ddeg, 0);
}

// 向后兼容：没有标定字段的旧 YAML 仍然能读，且等价于"不标定"
TEST(Servo18MapperTest, YamlWithoutCalibrationFieldsFallsBackToIdentity) {
    const auto path = write_temp_yaml(
        build_yaml(static_cast<int>(kChannelCount), false),
        "servo_map_without_calibration.yaml");

    const auto map = Servo18Mapper::load_map_from_yaml(path);

    ASSERT_EQ(map.size(), kChannelCount);
    for (const auto& entry : map) {
        EXPECT_EQ(entry.direction, 1);
        EXPECT_EQ(entry.offset_ddeg, 0);
    }
}

TEST(Servo18MapperTest, YamlRejectsInvalidDirection) {
    // direction 只能是 +1 或 -1；0 会静默地把输出全部清零，必须拦住
    const auto path = write_temp_yaml(
        build_yaml(static_cast<int>(kChannelCount), true, 0, 0),
        "servo_map_bad_direction.yaml");

    EXPECT_THROW(Servo18Mapper::load_map_from_yaml(path), std::runtime_error);
}

TEST(Servo18MapperTest, YamlRejectsWrongEntryCount) {
    // 少于 18 条意味着有通道没人写，必须拦住而不是让它保持 0
    const auto path = write_temp_yaml(build_yaml(3, true), "servo_map_too_short.yaml");

    EXPECT_THROW(Servo18Mapper::load_map_from_yaml(path), std::runtime_error);
}

TEST(Servo18MapperTest, YamlRejectsUnknownLegOrJoint) {
    std::string yaml = build_yaml(static_cast<int>(kChannelCount), false);
    // 把某一条的腿名改坏
    const auto pos = yaml.find("leg: lf");
    ASSERT_NE(pos, std::string::npos);
    yaml.replace(pos, 7, "leg: xx");

    const auto path = write_temp_yaml(yaml, "servo_map_bad_leg.yaml");

    EXPECT_THROW(Servo18Mapper::load_map_from_yaml(path), std::runtime_error);
}

// ---------------------------------------------------------------------------
// 关节命名规则：与 spider/scripts/generate_urdf.py 的跨语言契约
// ---------------------------------------------------------------------------
//
// 这条规则错了**不会报错**，只会让 RViz 里的腿一动不动——/joint_states 里
// 名字对不上的条目会被 robot_state_publisher 静默忽略，日志上看不出任何异常。
// 所以下面把格式和完整列表都钉死，而不是只测一两个样本。

TEST(Servo18MapperTest, JointStateNameFollowsPattern) {
    EXPECT_EQ(joint_state_name(LegId::LeftFront, JointId::Coxa), "lf_coxa_joint");
    EXPECT_EQ(joint_state_name(LegId::LeftFront, JointId::Femur), "lf_femur_joint");
    EXPECT_EQ(joint_state_name(LegId::LeftFront, JointId::Tibia), "lf_tibia_joint");
    EXPECT_EQ(joint_state_name(LegId::RightMiddle, JointId::Femur), "rm_femur_joint");
    EXPECT_EQ(joint_state_name(LegId::RightRear, JointId::Tibia), "rr_tibia_joint");
}

// 18 个名字必须两两不同：若有重名，后一个会静默覆盖前一个，
// 表现为"某两条腿永远同步动作"，非常难查。
TEST(Servo18MapperTest, JointStateNamesAreAllDistinct) {
    std::vector<std::string> names;
    for (const auto leg_id : kAllLegIds) {
        for (const auto joint_id : {JointId::Coxa, JointId::Femur, JointId::Tibia}) {
            names.push_back(joint_state_name(leg_id, joint_id));
        }
    }
    ASSERT_EQ(names.size(), 18u);

    auto sorted = names;
    std::sort(sorted.begin(), sorted.end());
    EXPECT_EQ(std::unique(sorted.begin(), sorted.end()), sorted.end());
}

// 腿名部分必须复用 leg_name()：腿名已经在日志和 servo_map.yaml 里作为键使用，
// 这里不能另起一套写法。
TEST(Servo18MapperTest, JointStateNameLegPartMatchesLegName) {
    for (const auto leg_id : kAllLegIds) {
        const auto prefix = leg_calc::leg_name(leg_id) + "_";
        for (const auto joint_id : {JointId::Coxa, JointId::Femur, JointId::Tibia}) {
            const auto name = joint_state_name(leg_id, joint_id);
            EXPECT_EQ(name.rfind(prefix, 0), 0u) << name;
            EXPECT_EQ(name.size() >= 6 ? name.substr(name.size() - 6) : name, "_joint") << name;
        }
    }
}
