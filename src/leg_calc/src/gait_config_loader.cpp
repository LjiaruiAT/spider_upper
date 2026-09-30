#include "leg_calc/gait_config_loader.hpp"

#include <fstream>
#include <stdexcept>

namespace leg_calc {

namespace {

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
}

// 命中 "key: <number>" 时写入 target 并返回 true。
// 非数字值在这里转成带字段名的报错——配置出错时最需要知道"是哪个字段的值不对"，
// 直接漏出 std::stod 的 "stod" 报错会让人去翻代码。
bool parse_number(const std::string& line, const std::string& key, double& target) {
    if (line.rfind(key, 0) != 0) {
        return false;
    }
    const std::string value = trim(line.substr(key.size()));
    try {
        target = std::stod(value);
    } catch (const std::exception&) {
        throw std::runtime_error("gait_params: '" + key + "' 的值不是数字：" + value);
    }
    return true;
}

}  // namespace

GaitConfig load_gait_config_from_yaml(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open gait params file: " + yaml_path);
    }

    // 用独立的布尔量记录"字段是否出现过"，而不是拿数值当哨兵：
    // step_height_m / 加速度上限都允许 0，用 0 判断"缺字段"会把合法的 0 误判。
    bool has_frequency = false;
    bool has_step_length = false;
    bool has_step_height = false;
    bool has_lateral_step = false;
    bool has_turn_step = false;
    bool has_linear_accel = false;
    bool has_angular_accel = false;

    // 从默认值起步，pattern 保持默认（由节点用 ROS 参数覆盖）。
    GaitConfig config;

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        if (trimmed == "gait_params:") {
            continue;
        }

        if (parse_number(trimmed, "frequency_hz:", config.frequency_hz)) {
            has_frequency = true;
            continue;
        }
        if (parse_number(trimmed, "step_length_m:", config.step_length_m)) {
            has_step_length = true;
            continue;
        }
        if (parse_number(trimmed, "step_height_m:", config.step_height_m)) {
            has_step_height = true;
            continue;
        }
        if (parse_number(trimmed, "lateral_step_m:", config.lateral_step_m)) {
            has_lateral_step = true;
            continue;
        }
        if (parse_number(trimmed, "turn_step_rad:", config.turn_step_rad)) {
            has_turn_step = true;
            continue;
        }
        if (parse_number(trimmed, "max_linear_accel_mps2:", config.max_linear_accel_mps2)) {
            has_linear_accel = true;
            continue;
        }
        if (parse_number(trimmed, "max_angular_accel_rps2:", config.max_angular_accel_rps2)) {
            has_angular_accel = true;
            continue;
        }
    }

    if (!(has_frequency && has_step_length && has_step_height && has_lateral_step &&
          has_turn_step && has_linear_accel && has_angular_accel)) {
        throw std::runtime_error(
            "gait_params: 缺少必需字段（需要 frequency_hz / step_length_m / step_height_m / "
            "lateral_step_m / turn_step_rad / max_linear_accel_mps2 / max_angular_accel_rps2 全部列出）：" +
            yaml_path);
    }

    // 数值合法性。只拦"明显错误"——不做"是否合理"的过度判断
    // （哪些值合理取决于机械能力，那是实机标定阶段的事）。
    if (config.frequency_hz <= 0.0) {
        throw std::runtime_error("gait_params: frequency_hz 必须为正数（步态周期不能为 0 或负）");
    }
    if (config.step_length_m <= 0.0 || config.lateral_step_m <= 0.0 || config.turn_step_rad <= 0.0) {
        throw std::runtime_error(
            "gait_params: 步长上限必须为正数（step_length_m / lateral_step_m / turn_step_rad）");
    }
    if (config.step_height_m < 0.0) {
        throw std::runtime_error("gait_params: step_height_m 不能为负（0 = 不抬腿，合法但罕见）");
    }
    // 加速度上限 **0 是合法的**：语义是"关闭速度平滑"，是对照实验的开关
    // （见 velocity_smoother.hpp）。只有负值才是配置错误。
    if (config.max_linear_accel_mps2 < 0.0 || config.max_angular_accel_rps2 < 0.0) {
        throw std::runtime_error("gait_params: 加速度上限不能为负（0 = 关闭速度平滑，是合法值）");
    }

    return config;
}

}  // namespace leg_calc
