#pragma once

#include "leg_calc/common_types.hpp"

#include <Eigen/Dense>
#include <string>

namespace leg_calc {

// 一个腿座的安装规格（描述**左侧**那条腿）。
// 右侧由镜像得到：x 不变、y 取负、yaw 取负——图纸本身就是镜像对称的。
struct LegMountSpec {
    double x_m{0.0};      // 髋关节轴在身体坐标系中的 x（前为正）
    double y_m{0.0};      // 髋关节轴在身体坐标系中的 y（左为正，这里填正值）
    double yaw_rad{0.0};  // 腿座绕竖直轴的朝向，0 = 腿朝身体正前方
};

// 六足的静态布局 + 单腿几何。
//
// 这是"配置层 -> 数学层"的边界结构：所有长度单位米、角度单位弧度，
// 没有 mm / deg 混进来（YAML 里是 mm / deg，加载时一次性换算）。
struct LegLayoutConfig {
    // 单腿三个铰链轴之间的距离
    double coxa_length_m{0.0};
    double femur_length_m{0.0};
    double tibia_length_m{0.0};

    // 归位姿态：足端在**腿局部坐标系**里的位置。
    //
    // 为什么用腿局部坐标而不是身体坐标：六条腿的局部坐标轴已经按各自的 yaw
    // 转到了朝外，所以"足端在髋轴外侧多远、下方多深"对六条腿是同一个描述，
    // 一个向量就够，不需要按腿分别配置。
    Eigen::Vector3d home_local_m{Eigen::Vector3d::Zero()};

    // 左侧三条腿的腿座
    LegMountSpec front;
    LegMountSpec middle;
    LegMountSpec rear;
};

// 按布局生成六条腿的安装位姿（body_T_leg）。
// 身体坐标系约定：x 前 / y 左 / z 上（见决策 D1）。
SpiderFrameBundle build_frame_bundle(const LegLayoutConfig& config);

// 从 YAML 读取布局配置。
// 文件里的长度字段单位是 mm、角度字段单位是 deg，读入后统一换算为 m / rad。
// 缺字段或格式错误会抛 std::runtime_error。
LegLayoutConfig load_leg_layout_from_yaml(const std::string& yaml_path);

}  // namespace leg_calc
