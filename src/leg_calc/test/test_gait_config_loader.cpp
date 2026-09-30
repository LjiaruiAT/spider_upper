// 步态参数加载测试
//
// 这个加载器是"配置即唯一真相"的守门人，重点锁定三件事：
//   1. 缺任何一个字段都必须拒绝——不做"缺了就用代码默认值"的兜底。
//      否则改了 YAML 却漏一个字段时，行为会悄悄按旧默认值跑。
//   2. 非法值拒绝（频率/步长必须为正、加速度不能为负），
//      但 **加速度 0 必须合法**——它是"关闭速度平滑"的对照实验开关。
//   3. 加载器**不碰 pattern**：步态模式是 ROS 参数，不是这个文件的事。
//
// 运行：colcon test --packages-select leg_calc

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

#include "leg_calc/gait_config_loader.hpp"
#include "leg_calc/gait_types.hpp"

namespace {

const char* kTestYamlPath = "/tmp/leg_calc_test_gait_params.yaml";

// 完整合法的文件内容：与 src/spider/config/gait_params.yaml 的默认值一致。
// 各用例都在它基础上"替换 / 删除一个字段"，这样被测字段就是唯一的变量。
const char* kFullYaml =
    "gait_params:\n"
    "  frequency_hz: 2.5\n"
    "  step_length_m: 0.04\n"
    "  step_height_m: 0.03\n"
    "  lateral_step_m: 0.02\n"
    "  turn_step_rad: 0.15\n"
    "  max_linear_accel_mps2: 0.4\n"
    "  max_angular_accel_rps2: 1.0\n";

void write_yaml(const std::string& content) {
    std::ofstream out(kTestYamlPath);
    out << content;
}

// 把完整 YAML 中的原字段行替换成 replacement（传空串 = 删除该字段）。
std::string replace_field(const std::string& original_line, const std::string& replacement) {
    std::string content(kFullYaml);
    const auto pos = content.find(original_line);
    EXPECT_NE(pos, std::string::npos) << "kFullYaml 里找不到字段：" << original_line;
    content.replace(pos, original_line.size(), replacement);
    return content;
}

}  // namespace

TEST(GaitConfigLoaderTest, LoadsAllValues) {
    write_yaml(kFullYaml);
    const auto config = leg_calc::load_gait_config_from_yaml(kTestYamlPath);

    EXPECT_NEAR(config.frequency_hz, 2.5, 1e-12);
    EXPECT_NEAR(config.step_length_m, 0.04, 1e-12);
    EXPECT_NEAR(config.step_height_m, 0.03, 1e-12);
    EXPECT_NEAR(config.lateral_step_m, 0.02, 1e-12);
    EXPECT_NEAR(config.turn_step_rad, 0.15, 1e-12);
    EXPECT_NEAR(config.max_linear_accel_mps2, 0.4, 1e-12);
    EXPECT_NEAR(config.max_angular_accel_rps2, 1.0, 1e-12);

    std::remove(kTestYamlPath);
}

// 加载器不碰 pattern——步态模式是 ROS 参数 `gait_pattern` 的事。
// 这条钉住边界：以后有人想把 pattern 塞进 YAML 时会先在这里撞墙，
// 从而必须先处理节点里"YAML 与 ROS 参数谁覆盖谁"的顺序问题。
TEST(GaitConfigLoaderTest, KeepsDefaultPattern) {
    write_yaml(kFullYaml);
    const auto config = leg_calc::load_gait_config_from_yaml(kTestYamlPath);
    EXPECT_EQ(config.pattern, leg_calc::GaitPattern::Tripod);
    std::remove(kTestYamlPath);
}

// 逐字段删除（其余保持合法）：每个字段缺失都必须被拒绝
TEST(GaitConfigLoaderTest, MissingAnyFieldIsRejected) {
    const char* field_lines[] = {
        "  frequency_hz: 2.5\n",
        "  step_length_m: 0.04\n",
        "  step_height_m: 0.03\n",
        "  lateral_step_m: 0.02\n",
        "  turn_step_rad: 0.15\n",
        "  max_linear_accel_mps2: 0.4\n",
        "  max_angular_accel_rps2: 1.0\n",
    };
    for (const char* omitted : field_lines) {
        write_yaml(replace_field(omitted, ""));  // 删掉这一个字段
        EXPECT_THROW(leg_calc::load_gait_config_from_yaml(kTestYamlPath), std::runtime_error)
            << "缺少字段 " << omitted << " 时没有被拒绝";
    }
    std::remove(kTestYamlPath);
}

TEST(GaitConfigLoaderTest, NonPositiveFrequencyIsRejected) {
    for (const char* bad_value : {"0.0", "-1.0"}) {
        write_yaml(replace_field(
            "  frequency_hz: 2.5\n",
            std::string("  frequency_hz: ") + bad_value + "\n"));
        EXPECT_THROW(leg_calc::load_gait_config_from_yaml(kTestYamlPath), std::runtime_error)
            << "frequency_hz = " << bad_value << " 时没有被拒绝";
    }
    std::remove(kTestYamlPath);
}

TEST(GaitConfigLoaderTest, NonPositiveStepLimitsAreRejected) {
    const std::pair<const char*, const char*> cases[] = {
        {"  step_length_m: 0.04\n", "  step_length_m: 0.0\n"},
        {"  lateral_step_m: 0.02\n", "  lateral_step_m: 0.0\n"},
        {"  turn_step_rad: 0.15\n", "  turn_step_rad: 0.0\n"},
    };
    for (const auto& [original, replacement] : cases) {
        write_yaml(replace_field(original, replacement));
        EXPECT_THROW(leg_calc::load_gait_config_from_yaml(kTestYamlPath), std::runtime_error)
            << "字段被置为 0 时没有被拒绝：" << replacement;
    }
    std::remove(kTestYamlPath);
}

// 加速度 0 **必须合法**：它是"关闭速度平滑"的对照实验开关。
// 这条防止加载器把合法的 0 误当成"缺字段"或"非法值"——
// 那会让对照实验做不了，而且故障表现是"配置改不动"，很难往上想。
TEST(GaitConfigLoaderTest, ZeroAccelerationIsAllowed) {
    write_yaml(
        "gait_params:\n"
        "  frequency_hz: 2.5\n"
        "  step_length_m: 0.04\n"
        "  step_height_m: 0.03\n"
        "  lateral_step_m: 0.02\n"
        "  turn_step_rad: 0.15\n"
        "  max_linear_accel_mps2: 0.0\n"
        "  max_angular_accel_rps2: 0.0\n");

    const auto config = leg_calc::load_gait_config_from_yaml(kTestYamlPath);
    EXPECT_DOUBLE_EQ(config.max_linear_accel_mps2, 0.0);
    EXPECT_DOUBLE_EQ(config.max_angular_accel_rps2, 0.0);

    std::remove(kTestYamlPath);
}

// 抬腿高度 0 合法（退化但可用）；负值拒绝
TEST(GaitConfigLoaderTest, ZeroStepHeightAllowedButNegativeIsRejected) {
    write_yaml(replace_field("  step_height_m: 0.03\n", "  step_height_m: 0.0\n"));
    EXPECT_DOUBLE_EQ(leg_calc::load_gait_config_from_yaml(kTestYamlPath).step_height_m, 0.0);

    write_yaml(replace_field("  step_height_m: 0.03\n", "  step_height_m: -0.01\n"));
    EXPECT_THROW(leg_calc::load_gait_config_from_yaml(kTestYamlPath), std::runtime_error);

    std::remove(kTestYamlPath);
}

// 非数字的值必须报"哪个字段"，而不是漏出 stod 的原始报错
TEST(GaitConfigLoaderTest, NonNumericValueIsRejected) {
    write_yaml(replace_field("  frequency_hz: 2.5\n", "  frequency_hz: abc\n"));
    EXPECT_THROW(leg_calc::load_gait_config_from_yaml(kTestYamlPath), std::runtime_error);
    std::remove(kTestYamlPath);
}

TEST(GaitConfigLoaderTest, MissingFileIsRejected) {
    EXPECT_THROW(
        leg_calc::load_gait_config_from_yaml("/tmp/leg_calc_gait_params_does_not_exist.yaml"),
        std::runtime_error);
}
