#pragma once

#include "leg_calc/gait_types.hpp"

#include <string>

namespace leg_calc {

// 从 YAML 读取步态参数（GaitConfig 的数值部分）。
//
// 为什么单独一个文件：这些参数原来是 GaitConfig 的 C++ 默认值，改一个数字
// 就得重新编译。它们直接决定"机器人能怎么动"（周期、步长上限、抬腿高度、
// 加速度上限），属于配置，应该和机械几何一样从 YAML 来。
//
// 边界：
//   · 本函数只负责 7 个**数值**字段；步态模式 pattern 不在这里——
//     它是 ROS 参数 `gait_pattern`（支持启动时选择，见节点）。
//   · 7 个字段**全部必需**：缺任何一个都抛异常。不做"缺了就用代码默认值"
//     的兜底——那样配置就不是唯一真相，改 YAML 时漏一个字段会悄悄按旧默认值跑。
//   · 非法值直接抛异常（频率 <= 0、步长 <= 0、加速度 < 0），宁可启动失败。
//     注意加速度 **0 是合法的**：语义是"关闭速度平滑"（对照实验开关）。
//   · 字段名与 GaitConfig 成员一一对应，单位也是 m / s / Hz——
//     不做 mm 换算：这些不是图纸数据，字段名后缀已经说明单位。
GaitConfig load_gait_config_from_yaml(const std::string& yaml_path);

}  // namespace leg_calc
