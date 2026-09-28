#!/usr/bin/env python3
"""从机械图纸导出 URDF 用的 mesh：body / coxa / femur / tibia。

**这不是构建步骤，是一次性的工具**：图纸不变就不用再跑。
产物已经提交在 `model/` 里，`generate_urdf.py` 只读它们。

用法（需要 FreeCAD，见下）。**必须先 cd 到 `src/spider/`**，产物写到 `./model/`：
    conda create -y -n freecad -c conda-forge freecad
    cd src/spider
    ~/miniconda3/envs/freecad/bin/freecadcmd -c "exec(open('scripts/export_cad_meshes.py').read())"

--------------------------------------------------------------------------
为什么必须用 FreeCAD，而不能自己解析 .FCStd
--------------------------------------------------------------------------
.FCStd 是个 zip，里面是 Document.xml + 一堆 `.brp`。但 `Placement` 是**逐层累乘**的
（`App::Part` 容器自己也带旋转），手写 XML 解析极容易把旋转方向算反。
本项目就踩过：四元数转矩阵时 y 的符号错了 → 零件看起来"横"着 → 误以为腿沿 y 方向，
白绕了一大圈才发现是解析错误。用官方 API 的 `getGlobalPlacement()` 没有这个风险。

apt 里的 freecad 是 0.19，**打不开 1.x 格式的图纸**（XML 结构变了），所以用 conda-forge。

--------------------------------------------------------------------------
图纸的三个关键事实（决定了脚本怎么写）
--------------------------------------------------------------------------
① **图纸里没有"装好的腿"**。`Arm` / `MiddleArm` / `ArmTip` 是三个各自独立建模、
   朝向互不相干的子装配，各自摆在不同位置。所以不能直接读全局坐标当装配位置。

② **`ArmTip` 容器的原点就是膝关节**。它的全局 x = 149.80 = 60.3 + 89.5
   （coxa_length + femur_length），精确吻合。抓住这个锚点，就不用去反推关节在哪儿。
   图纸把腿伸直摆着，所以三个关节共线，y/z 相同 → 关节在
   (0 / 60.3 / 149.80, -14.77, 12.15)。

③ **`HorizontalConnector` 等与 `Arm` 是平级容器**，不是 `Arm` 的子对象。
   只导 `Arm` 会漏掉髋的水平连接件。

--------------------------------------------------------------------------
两个容易把几何搞错的点
--------------------------------------------------------------------------
· `App::Clone` / `Slice` 是**派生体**：`ArmHolderBase`、`Slice005`、
  `TopHolder+BottomHolder` 三者几何完全重合。不去重的话体积会翻几倍。
  这里按"体积 + 全局包围盒"判重。
· `App::Part` 对象带 `Extensions="True"` 属性。如果改用 XML 正则匹配
  `<Object name="X">`，会恰好漏掉所有装配容器——它们的属性里都有这一项。
"""

import os
import struct
import sys

import FreeCAD
import MeshPart
import Part

LEG_DWG = os.path.expanduser(
    "~/Desktop/exist_urdf/Hexapod/cad/freecad/Hexapod-Leg_v2.FCStd"
)
STL_DIR = os.path.expanduser("~/Desktop/exist_urdf/hardware/stl")

# ⚠ freecadcmd -c "exec(...)" 时**没有 `__file__`**（脚本不是被当模块加载的），
# 所以要退回到当前工作目录——脚本约定从 src/spider/ 下运行。
#
# 两种情况下"包根"的算法不同：
#   __file__ 可用 → scripts/ 的上一级
#   不可用       → 当前工作目录本身就是包根
try:
    _BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
except NameError:
    _BASE = os.getcwd()
OUT_DIR = os.path.join(_BASE, "model")

MM_TO_M = 0.001

# 每条连杆由哪些容器组成。HorizontalConnector 等与 Arm 平级，必须一起收。
LINKS = {
    "coxa": ("Part010", "Part003", "Part001", "Part002", "Part005"),
    "femur": ("Part006",),
    "tibia": ("Part009",),
}

# 各连杆的关节在图纸全局坐标里的位置（mm），见上面 ② 的说明
JOINTS = {
    "coxa": (0.0, -14.77, 12.15),
    "femur": (60.3, -14.77, 12.15),
    "tibia": (149.80, -14.77, 12.15),
}

# ---------------------------------------------------------------------------
# 机身
# ---------------------------------------------------------------------------
# 机身是**上下两层**结构，所以必须整体从机身图纸导出，不能只挑几个 STL：
# 图纸里的 Body003(PCB_Connector) 与 pcb_chasis_holder.stl 的 z 中心差 15mm，
# 混用两边的坐标会错位。同一个文档里导，各层天然共享坐标系。
#
# 结构件就是那 5 个 Body*（其余 Part__Feature* 是电池 / 树莓派 / LED / 螺钉）：
#     Body        底板             y -73.59 ~ -53.59   164.0×20.0×180.8
#     Body002     支柱(Standoffs)  y -67.59 ~ -39.59   116.6×28.0×154.8
#     Body001     电池架           y -47.59 ~ -42.59    77.3× 5.0×119.6
#     Body003     PCB 托板         y -39.00 ~ -16.00   164.0×23.0×181.1
#     Body005     上盖             y -16.00 ~  19.00    98.6×35.0×138.2
BODY_DWG = os.path.expanduser(
    "~/Desktop/exist_urdf/hardware/freecad/Hexapod-Chassis.FCStd"
)
BODY_OBJECTS = ("Body", "Body001", "Body002", "Body003", "Body005")

PLATE_CENTER = (0.0, -63.59, 0.0)     # 底板在图纸坐标里的中心
PLATE_HALF_THICK = 10.0               # 底板厚 20mm 的一半
MOUNT_BOTTOM_Z = -21.56               # 髋座底面相对髋关节轴的高度（mm，实测）

# 机壳自身坐标 → URDF 机身帧（x 前 / y 左 / z 上）：
#     chassis x → body y     （机壳 156 那条边 → 左右）
#     chassis y → body z     （板法线 → 上）
#     chassis z → body x     （机壳 174.78 那条边 → 前后）
# 依据：机壳轮廓沿 ±x 的极值点跨度只有 23.3mm（两个窄凸耳），沿 ±z 有 105.1mm（整条长边）。
# **窄凸耳朝的就是中腿**（中腿正对侧方伸出）→ 所以 chassis x 对应 body y。
BODY_ROT = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def xyz(value):
    """坐标可能是 Base.Vector，也可能是 (x,y,z) 元组——FreeCAD 不同路径不一致。"""
    try:
        return value.x, value.y, value.z
    except AttributeError:
        return value[0], value[1], value[2]


def collect_final_parts(obj, out):
    """收集子树里的**最终实体**。

    PartDesign::Body 就是成品；App::Part / 组继续下钻；
    App::Clone / Slice / Binder 是派生体，跳过（否则几何会叠加）。
    """
    group = getattr(obj, "Group", None)
    if not group:
        try:
            if obj.Shape.Volume > 0:
                out.append(obj)
        except Exception:
            pass
        return
    for child in group:
        tid = child.TypeId
        if tid.startswith("App::") and tid not in ("App::Part", "App::DocumentObjectGroup"):
            continue
        if tid in ("PartDesign::Body", "Part::Feature"):
            try:
                if child.Shape.Volume > 0:
                    out.append(child)
            except Exception:
                pass
            continue
        if tid in ("App::Part", "App::DocumentObjectGroup"):
            collect_final_parts(child, out)


def dedup(shapes):
    """按"体积 + 全局包围盒"去掉几何重合的派生体。"""
    seen, keep = set(), []
    for shape in shapes:
        try:
            box = shape.BoundBox
            key = (round(shape.Volume, 2), round(box.XMin, 2), round(box.YMin, 2),
                   round(box.ZMin, 2), round(box.XLength, 2), round(box.YLength, 2),
                   round(box.ZLength, 2))
        except Exception:
            continue
        if key in seen:
            continue
        seen.add(key)
        keep.append(shape)
    return keep


def write_binary_stl(path, mesh, header):
    """写二进制 STL。

    FreeCAD 的 `Shape.exportStl()` 只有 ASCII 且用最细的离散精度——
    单条髋就 157 MB，URDF 用不了。所以自己走 MeshPart + 手动写二进制。
    """
    with open(path, "wb") as fh:
        fh.write(header.encode()[:80].ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(mesh.Facets)))
        for facet in mesh.Facets:
            points = facet.Points
            if len(points) != 3:
                continue
            fh.write(struct.pack("<3f", *xyz(facet.Normal)))
            for point in points:
                fh.write(struct.pack("<3f", *xyz(point)))
            fh.write(struct.pack("<H", 0))


def export_links(out_dir):
    doc = FreeCAD.openDocument(LEG_DWG)
    by_name = {obj.Name: obj for obj in doc.Objects}

    for label, roots in LINKS.items():
        parts = []
        for root in roots:
            collect_final_parts(by_name[root], parts)

        shapes = []
        for part in parts:
            shape = part.Shape.copy()
            shape.transformShape(part.getGlobalPlacement().toMatrix())
            shapes.append(shape)
        shapes = dedup(shapes)

        jx, jy, jz = JOINTS[label]
        moved = []
        for shape in shapes:
            s = shape.copy()
            # ① 平移到关节帧（此时还是 mm）
            s.translate(FreeCAD.Vector(-jx, -jy, -jz))
            # ② 换算成米。URDF 的长度单位是米，而 STL 不带单位信息，
            #    所以必须把单位烘进 mesh，否则 RViz 里会大 1000 倍。
            scale = FreeCAD.Matrix()
            scale.scale(MM_TO_M, MM_TO_M, MM_TO_M)
            s.transformShape(scale)
            moved.append(s)

        compound = Part.makeCompound(moved)
        mesh = MeshPart.meshFromShape(Shape=compound, LinearDeflection=0.00025,
                                      AngularDeflection=0.5, Relative=False)
        path = os.path.join(out_dir, f"{label}.stl")
        write_binary_stl(path, mesh, f"spider_upper {label} link mesh (SI units, meters)")
        box = compound.BoundBox
        print(f"{label:6s} 零件{len(moved):>3}  x {box.XMin:8.4f}~{box.XMax:8.4f}  "
              f"y {box.YMin:8.4f}~{box.YMax:8.4f}  z {box.ZMin:8.4f}~{box.ZMax:8.4f} m  "
              f"{len(mesh.Facets):>6} 面")


def mesh_to_array(mesh):
    """FreeCAD 的 Mesh → (N, 3, 3) 的 numpy 三角形数组。"""
    import numpy as np

    tris = []
    for facet in mesh.Facets:
        points = facet.Points
        if len(points) == 3:
            tris.append([xyz(p) for p in points])
    return np.array(tris, dtype=float)


def export_body(out_dir):
    """机身：上下两层结构（底板 / 支柱 / 电池架 / PCB 托板 / 上盖）。

    只在"取几何"这一步用 FreeCAD（`getGlobalPlacement()` 把各层放到图纸的装配位置），
    之后的旋转/平移/单位换算统一在 numpy 里做——**不碰 FreeCAD 的 Matrix API**，
    它的下标约定容易写错，而写错了不会报错，只会让机身整体歪掉。
    """
    import numpy as np

    doc = FreeCAD.openDocument(BODY_DWG)
    by_name = {obj.Name: obj for obj in doc.Objects}

    rot = np.array(BODY_ROT)
    center = np.array(PLATE_CENTER)
    target = np.array([0.0, 0.0, MOUNT_BOTTOM_Z - PLATE_HALF_THICK])
    # 相对底板中心 → 旋转 → 落到目标位置 → 换算成米。
    # 别再额外加回 center：那样等于整组又平移一次，会整体抬高 63.59mm。
    def to_body(points):
        return ((rot @ (points - center).T).T + target) * MM_TO_M

    merged = []
    for name in BODY_OBJECTS:
        obj = by_name.get(name)
        if obj is None:
            print(f"  ⚠ 图纸里找不到 {name}，跳过")
            continue
        shape = obj.Shape.copy()
        shape.transformShape(obj.getGlobalPlacement().toMatrix())
        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.25,
                                      AngularDeflection=0.5, Relative=False)
        tris = mesh_to_array(mesh)
        merged.append(to_body(tris.reshape(-1, 3)).reshape(-1, 3, 3))
        print(f"  {name:<10} {obj.Label[:16]:<18} {len(tris):>6} 面")

    tris = np.vstack(merged)

    # 法线自己算：旋转后原始法线不再可靠，而且 STL 的法线本来就有很多是错的
    path = os.path.join(out_dir, "body.stl")
    with open(path, "wb") as fh:
        fh.write(b"spider_upper body mesh (SI units, meters)".ljust(80, b"\0"))
        fh.write(struct.pack("<I", len(tris)))
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            length = np.linalg.norm(nrm)
            nrm = nrm / length if length > 1e-12 else np.zeros(3)
            fh.write(struct.pack("<3f", *nrm))
            for p in t:
                fh.write(struct.pack("<3f", *p))
            fh.write(struct.pack("<H", 0))

    lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
    print(f"body   层数{len(merged):>3}  x {lo[0]:8.4f}~{hi[0]:8.4f}  "
          f"y {lo[1]:8.4f}~{hi[1]:8.4f}  z {lo[2]:8.4f}~{hi[2]:8.4f} m  "
          f"{len(tris):>6} 面")


def main():
    out_dir = os.path.abspath(OUT_DIR)
    os.makedirs(out_dir, exist_ok=True)
    print(f"输出目录: {out_dir}\n")
    export_links(out_dir)
    export_body(out_dir)
    print("\n完成。URDF 里这些 mesh 已经在各自的连杆帧、单位是米，"
          "所以 <visual>/<collision> 不要写 <origin> 和 scale。")


# `freecadcmd -c "exec(open(...).read())"` 时 __name__ 不是 "__main__"，
# 所以两个条件都判一下。
if __name__ in ("__main__", "builtins"):
    main()
