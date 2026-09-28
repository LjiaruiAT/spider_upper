#include "leg_calc/leg_layout.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace leg_calc {

namespace {

constexpr double kMmToM = 0.001;
constexpr double kDegToRad = M_PI / 180.0;

std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t");
    return text.substr(first, last - first + 1);
}

// 命中 "key: <number>" 时写入 target 并返回 true。
bool parse_number(const std::string& line, const std::string& key, double& target) {
    if (line.rfind(key, 0) != 0) {
        return false;
    }
    target = std::stod(trim(line.substr(key.size())));
    return true;
}

// 命中 "key: [a, b, c]" 时写入 target 并返回 true。
bool parse_vector3(const std::string& line, const std::string& key, Eigen::Vector3d& target) {
    if (line.rfind(key, 0) != 0) {
        return false;
    }
    std::string rest = trim(line.substr(key.size()));
    if (rest.size() < 2 || rest.front() != '[' || rest.back() != ']') {
        throw std::runtime_error("leg_params: '" + key + "' expects [a, b, c]");
    }
    rest = rest.substr(1, rest.size() - 2);

    std::stringstream stream(rest);
    std::string item;
    int index = 0;
    while (std::getline(stream, item, ',')) {
        if (index >= 3) {
            throw std::runtime_error("leg_params: '" + key + "' has too many components");
        }
        target(index++) = std::stod(trim(item));
    }
    if (index != 3) {
        throw std::runtime_error("leg_params: '" + key + "' expects exactly 3 components");
    }
    return true;
}

}  // namespace

SpiderFrameBundle build_frame_bundle(const LegLayoutConfig& config) {
    SpiderFrameBundle bundle;
    bundle.body_frame.frame_id = "spider_base";

    for (const auto leg_id : kAllLegIds) {
        // yaw 的意义：腿局部坐标系的 x 轴指向"腿的外侧"。
        // 中腿要朝正侧方（±90°）、前腿朝斜前、后腿朝斜后，所以每条腿的 yaw 都不同。
        // 如果没有这个 yaw（全填 0），六条腿的 x 轴都朝身体前方，
        // 中腿就只能靠 coxa 关节转 90° 才能伸到侧面——那已经贴着限位了。
        const bool is_left = is_left_leg(leg_id);
        const double side = is_left ? 1.0 : -1.0;

        const LegMountSpec* spec = nullptr;
        switch (leg_id) {
        case LegId::LeftFront:
        case LegId::RightFront:
            spec = &config.front;
            break;
        case LegId::LeftMiddle:
        case LegId::RightMiddle:
            spec = &config.middle;
            break;
        default:
            spec = &config.rear;
            break;
        }

        // 镜像：x 不变，y 和 yaw 取负
        const Eigen::Vector3d origin(spec->x_m, side * spec->y_m, 0.0);
        bundle.leg_mounts[leg_index(leg_id)] = make_leg_mount_pose(leg_id, origin, side * spec->yaw_rad);
    }

    return bundle;
}

LegLayoutConfig load_leg_layout_from_yaml(const std::string& yaml_path) {
    std::ifstream input(yaml_path);
    if (!input.is_open()) {
        throw std::runtime_error("Failed to open leg params file: " + yaml_path);
    }

    LegLayoutConfig config;
    // 用 mm / deg 的临时量接收，最后统一换算——这样能保证不会漏掉某一次换算。
    double home_local_mm[3] = {0.0, 0.0, 0.0};
    bool has_home_local = false;

    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        Eigen::Vector3d home;
        if (parse_vector3(trimmed, "home_local_mm:", home)) {
            home_local_mm[0] = home.x();
            home_local_mm[1] = home.y();
            home_local_mm[2] = home.z();
            has_home_local = true;
            continue;
        }

        double value = 0.0;
        if (parse_number(trimmed, "coxa_length_mm:", value)) {
            config.coxa_length_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "femur_length_mm:", value)) {
            config.femur_length_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "tibia_length_mm:", value)) {
            config.tibia_length_m = value * kMmToM;
            continue;
        }

        if (parse_number(trimmed, "front_mount_x_mm:", value)) {
            config.front.x_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "front_mount_y_mm:", value)) {
            config.front.y_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "front_yaw_deg:", value)) {
            config.front.yaw_rad = value * kDegToRad;
            continue;
        }

        if (parse_number(trimmed, "middle_mount_x_mm:", value)) {
            config.middle.x_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "middle_mount_y_mm:", value)) {
            config.middle.y_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "middle_yaw_deg:", value)) {
            config.middle.yaw_rad = value * kDegToRad;
            continue;
        }

        if (parse_number(trimmed, "rear_mount_x_mm:", value)) {
            config.rear.x_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "rear_mount_y_mm:", value)) {
            config.rear.y_m = value * kMmToM;
            continue;
        }
        if (parse_number(trimmed, "rear_yaw_deg:", value)) {
            config.rear.yaw_rad = value * kDegToRad;
            continue;
        }
    }

    if (!has_home_local) {
        throw std::runtime_error("leg_params: missing 'home_local_mm' in " + yaml_path);
    }
    if (config.coxa_length_m <= 0.0 || config.femur_length_m <= 0.0 || config.tibia_length_m <= 0.0) {
        throw std::runtime_error(
            "leg_params: coxa/femur/tibia_length_mm must all be positive in " + yaml_path);
    }

    config.home_local_m = Eigen::Vector3d(home_local_mm[0], home_local_mm[1], home_local_mm[2]) * kMmToM;
    return config;
}

}  // namespace leg_calc
