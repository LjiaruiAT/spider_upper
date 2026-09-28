#!/usr/bin/env python3
"""从 leg_params.yaml 生成 URDF（给 RViz / robot_state_publisher 用）。

用法：
    ./generate_urdf.py > spider.urdf
    ./generate_urdf.py --leg-params /path/to/leg_params.yaml

--------------------------------------------------------------------------
为什么是"生成"而不是"手写一个 URDF 文件"
--------------------------------------------------------------------------
手写 URDF 需要把六个腿座的 x / y / yaw、左右镜像规则各抄一遍，
24 个 link、18 个 joint 的 origin 全靠手算——那是 18 个可以出错的地方，
而且错了**不会报错**，只会让 RViz 里的腿装在奇怪的位置，你还得肉眼去找。

这里全部从 leg_params.yaml 算出来，于是"配置"仍然只有一处真相：
和 leg_chain.cpp / leg_layout.cpp 读的是同一份文件、同一套规则。

--------------------------------------------------------------------------
关节命名规则（必须与 leg_calc 发布的 /joint_states 一致）
--------------------------------------------------------------------------
    {leg}_{joint}_joint         leg   ∈ lf lm lr rf rm rr
                                joint ∈ coxa femur tibia

    leg_calc 侧的对应实现在 common_types.hpp 的 leg_name()。
    两处都对不上时 RViz 里腿不会动（但也不会报错），所以这里刻意用了最短
    最直白的规则，减少"两边写法不同"的可能。

--------------------------------------------------------------------------
坐标系与符号约定（与 leg_calc 完全一致）
--------------------------------------------------------------------------
  · URDF 的长度单位是**米**；leg_params.yaml 是毫米，这里换算。
  · 腿局部坐标系：原点在髋关节轴线上，x 沿 coxa 指向这条腿的**外侧**，z 向上。
  · 三个关节轴：coxa 绕 **z**、femur 绕 **y**、tibia 绕 **y**。
    KDL 侧用的是 RotZ / RotY，同样是右手系绕轴旋转，
    所以 IK 解出的角度可以 **1:1 直接填进 /joint_states，不需要换符号**。
    —— 这一条如果错了，RViz 里的腿会朝反方向动，很难看出是符号问题。

--------------------------------------------------------------------------
连杆的视觉 mesh
--------------------------------------------------------------------------
  · 三条连杆用模型文件 `model/{coxa,femur,tibia}.stl`，从机械图纸导出。
  · mesh 的**坐标原点就是该连杆的关节**，单位已经是**米**——所以 URDF 里
    <visual> 不需要 <origin>，也不需要 scale。这两个地方任何一处写错，
    RViz 里都会表现成"mesh 飘在关节外面"，而且不会报错。
  · 导出方法：`model/README.md` 的"连杆 / 机身 mesh 是怎么来的"一节（需要 FreeCAD）。
  · 机身用 `model/body.stl`（图纸机身的 5 个结构件，上下两层），在同一机身帧里。
  · 仍没有 <inertial>：RViz 显示不需要，接 Gazebo 时才必须补。
"""

import argparse
import math
import os
import sys

# 机身 mesh：图纸机身的 5 个结构件（上下两层），已在机身帧里、单位米。
BODY_MESH = "body.stl"

# 三条连杆的 mesh 文件名。它们在 model/ 下，原点在各自关节、单位是米。
LINK_MESHES = {
    "coxa": "coxa.stl",
    "femur": "femur.stl",
    "tibia": "tibia.stl",
}

# 关节限位里的 effort / velocity。URDF 对 revolute 关节要求这两个字段。
# 数值取自 MG996R 厂商标称（11 kg·cm ≈ 1.08 N·m；0.17 s/60° ≈ 6.2 rad/s），
# **未经实测**。它们不影响显示，只影响以后接动力学仿真时的可信度。
JOINT_EFFORT_NM = 1.08
JOINT_VELOCITY_RAD_S = 6.2

# 颜色（RViz 里用）。让六条腿的关节层级一眼可辨。
COLOR_BODY = "0.55 0.55 0.58 1"
COLOR_COXA = "0.85 0.35 0.30 1"
COLOR_FEMUR = "0.35 0.75 0.40 1"
COLOR_TIBIA = "0.35 0.55 0.90 1"

# 六条腿：(短名, 左右符号, 配置里的腿座键)
# 顺序与 leg_calc 的 kAllLegIds 一致：lf lm lr rf rm rr
LEGS = [
    ("lf", +1.0, "front"),
    ("lm", +1.0, "middle"),
    ("lr", +1.0, "rear"),
    ("rf", -1.0, "front"),
    ("rm", -1.0, "middle"),
    ("rr", -1.0, "rear"),
]

JOINTS = ("coxa", "femur", "tibia")


def fmt(value):
    """紧凑地格式化一个浮点数，避免 URDF 里出现一堆 0.000000。"""
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return text if text not in ("", "-") else "0"


def parse_args():
    parser = argparse.ArgumentParser(description="从 leg_params.yaml 生成 URDF")
    parser.add_argument(
        "--leg-params",
        default=None,
        help="leg_params.yaml 的路径；默认从 spider 包的 share 目录找",
    )
    return parser.parse_args()


def resolve_leg_params_path(explicit):
    if explicit:
        return explicit
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        sys.exit(
            "找不到 leg_params.yaml：没有指定 --leg-params，"
            "且 ament_index_python 不可用（请先 source ROS 环境）"
        )
    return os.path.join(get_package_share_directory("spider"), "config", "leg_params.yaml")


def load_params(path):
    """读 key: value 形式的简单 YAML。

    刻意不引入 PyYAML：这个文件只有一层、全是标量，手写解析反而少一个依赖，
    而且与 leg_layout.cpp 的做法保持一致（那边也是手写的）。
    """
    if not os.path.isfile(path):
        sys.exit(f"找不到配置文件：{path}")

    raw = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, value = line.partition(":")
            raw[key.strip()] = value.strip()

    def number(key):
        if key not in raw:
            sys.exit(f"leg_params.yaml 缺少字段：{key}")
        try:
            return float(raw[key])
        except ValueError:
            sys.exit(f"leg_params.yaml 的 {key} 不是数字：{raw[key]!r}")

    def pair(key, suffix):
        # front_mount_x_mm: 96.52  ->  {"x": 0.09652, ...} 的 x 分量
        return number(f"{key}_mount_{suffix}_mm") * 0.001

    params = {
        "coxa": number("coxa_length_mm") * 0.001,
        "femur": number("femur_length_mm") * 0.001,
        "tibia": number("tibia_length_mm") * 0.001,
        "mounts": {
            key: {
                "x": pair(key, "x"),
                "y": pair(key, "y"),
                "yaw": number(f"{key}_yaw_deg") * math.pi / 180.0,
            }
            for key in ("front", "middle", "rear")
        },
        "limits": {
            joint: (
                number(f"{joint}_min_deg") * math.pi / 180.0,
                number(f"{joint}_max_deg") * math.pi / 180.0,
            )
            for joint in JOINTS
        },
    }
    return params


def resolve_mount(params, leg_short, side, spec_key):
    """把左侧腿座规格镜像成这条腿的实际位姿。

    镜像规则（与 leg_layout.cpp 的 build_frame_bundle 完全一致）：
        x 不变、y 取负、yaw 取负
    """
    spec = params["mounts"][spec_key]
    return spec["x"], side * spec["y"], side * spec["yaw"]


MATERIALS = {
    "body": COLOR_BODY,
    "coxa": COLOR_COXA,
    "femur": COLOR_FEMUR,
    "tibia": COLOR_TIBIA,
}


def visual_material(name):
    """渲染在 <visual> 内部的 <material>。

    ⚠ 位置必须是 <visual> 的**子元素**，不能直接挂在 <link> 下。
    挂错了**不会报错**——urdf 解析器会直接忽略它，然后 RViz 用默认的红色画
    所有 link。表现为"机器人画出来了，但全是红的"，看不出是材质问题。
    （第一次就是这么写错的，靠截图才发现。）
    """
    return (
        f'      <material name="{name}">\n'
        f'        <color rgba="{MATERIALS[name]}"/>\n'
        f'      </material>\n'
    )


def joint_xml(name, parent, child, xyz, rpy, axis, lower, upper):
    return (
        f'  <joint name="{name}" type="revolute">\n'
        f'    <parent link="{parent}"/>\n'
        f'    <child link="{child}"/>\n'
        f'    <origin xyz="{xyz}" rpy="{rpy}"/>\n'
        f'    <axis xyz="{axis}"/>\n'
        f'    <limit lower="{fmt(lower)}" upper="{fmt(upper)}"'
        f' effort="{JOINT_EFFORT_NM}" velocity="{JOINT_VELOCITY_RAD_S}"/>\n'
        f'  </joint>\n'
    )


def link_xml(name, mesh_key, material_name):
    """一条连杆：视觉与碰撞都用图纸导出的 mesh。

    ⚠ **不写 `<origin>`，也不写 `scale`。**
    mesh 的原点已经落在该连杆的关节上，单位已经换算成米。
    在这里多写一个 origin（哪怕写 0 0 0 之外的值）或多写一个 scale，
    RViz 里就会表现为"mesh 飘在关节外面"，而且**不报任何错**——只能靠肉眼发现。
    """
    geometry = (
        f'      <geometry>\n'
        f'        <mesh filename="package://spider/model/{LINK_MESHES[mesh_key]}"/>\n'
        f'      </geometry>\n'
    )
    return (
        f'  <link name="{name}">\n'
        f'    <visual>\n'
        f'{geometry}'
        f'{visual_material(material_name)}'
        f'    </visual>\n'
        f'    <collision>\n'
        f'{geometry}'
        f'    </collision>\n'
        f'  </link>\n'
    )


def body_xml(params):
    """机身：图纸机身的 5 个结构件（底板 / 支柱 / 电池架 / PCB 托板 / 上盖）。

    与连杆同理：mesh 已在机身帧里、单位是米，所以不写 origin / scale。
    底板顶面落在 z = -21.56mm —— 实测髋座（coxa_mount）底面就在髋关节轴
    下方这个高度，所以底板和六条腿的髋座是贴合的。

    ⚠ 机身是**上下两层**的。只导底板会得到"上半部分是空的"
    （整机照片上看不出第二层，但 CAD 里有托板和支柱）。
    """
    _ = params          # 机身几何不再由腿座位置推算
    geometry = (
        f'      <geometry>\n'
        f'        <mesh filename="package://spider/model/{BODY_MESH}"/>\n'
        f'      </geometry>\n'
    )
    return (
        f'  <link name="spider_base">\n'
        f'    <visual>\n'
        f'{geometry}'
        f'{visual_material("body")}'
        f'    </visual>\n'
        f'    <collision>\n'
        f'{geometry}'
        f'    </collision>\n'
        f'  </link>\n'
    )


def leg_xml(params, leg_short, side, spec_key):
    coxa = params["coxa"]
    femur = params["femur"]
    tibia = params["tibia"]
    limits = params["limits"]
    x, y, yaw = resolve_mount(params, leg_short, side, spec_key)

    out = []
    out.append(f'  <!-- {leg_short} -->\n')

    # 关节 1：coxa 绕竖直轴转向。腿座本身朝向已由 yaw 表达，
    # 所以这里 rpy 的第三个分量就是 yaw。
    out.append(
        joint_xml(
            f"{leg_short}_coxa_joint",
            "spider_base",
            f"{leg_short}_coxa_link",
            f"{fmt(x)} {fmt(y)} 0",
            f"0 0 {fmt(yaw)}",
            "0 0 1",
            *limits["coxa"],
        )
    )
    out.append(link_xml(f"{leg_short}_coxa_link", "coxa", "coxa"))

    # 关节 2：femur 俯仰，位于 coxa 杆末端，绕局部 y。
    out.append(
        joint_xml(
            f"{leg_short}_femur_joint",
            f"{leg_short}_coxa_link",
            f"{leg_short}_femur_link",
            f"{fmt(coxa)} 0 0",
            "0 0 0",
            "0 1 0",
            *limits["femur"],
        )
    )
    out.append(link_xml(f"{leg_short}_femur_link", "femur", "femur"))

    # 关节 3：tibia（膝盖），位于 femur 杆末端，同样绕局部 y。
    # 与 leg_chain.cpp 一致：q1 / q2 同号叠加。
    out.append(
        joint_xml(
            f"{leg_short}_tibia_joint",
            f"{leg_short}_femur_link",
            f"{leg_short}_tibia_link",
            f"{fmt(femur)} 0 0",
            "0 0 0",
            "0 1 0",
            *limits["tibia"],
        )
    )
    out.append(link_xml(f"{leg_short}_tibia_link", "tibia", "tibia"))

    return "".join(out)


def generate(params, source_path):
    parts = [
        '<?xml version="1.0"?>\n',
        "<!--\n",
        "  本文件由 spider/scripts/generate_urdf.py 从 leg_params.yaml 生成，\n",
        "  请不要手工修改——改了会在下次生成时被覆盖。\n",
        f"  来源：{source_path}\n",
        "-->\n",
        '<robot name="spider">\n',
        body_xml(params),
    ]
    for leg_short, side, spec_key in LEGS:
        parts.append(leg_xml(params, leg_short, side, spec_key))
    parts.append("</robot>\n")
    return "".join(parts)


def main():
    args = parse_args()
    path = resolve_leg_params_path(args.leg_params)
    params = load_params(path)
    sys.stdout.write(generate(params, path))


if __name__ == "__main__":
    main()
