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
① **图纸就是一条装好的腿**（伸直摆着）。`Arm` / `MiddleArm` / `ArmTip`
   三个子装配在全局坐标下于关节处自然衔接。

② **关节轴 = 舵机输出轴的圆柱面**，不是容器的原点。

   ⚠ 曾经把 `ArmTip` 容器的原点 (149.80, -14.77, 12.15) 当成膝关节——
   那只是建模参考点。用 FreeCAD 量舵机输出轴（沿轴线的最大圆柱，R10）
   得到的**真实旋转轴**是：

       髋（竖直轴）  过 (10.10, -3.17)，沿 z     ← 髋舵机的输出轴
       股（沿 y）    过 x=70.40, z=15.33         ← 股舵机 MG996R013 的输出轴
       膝（沿 y）    过 x=159.90, z=15.33        ← 膝舵机 MG996R014 的输出轴

   两个**精确到 0.01mm** 的吻合可以相互印证：
       股轴 − 髋轴 = 70.40 − 10.10 = 60.30 = COXA_LENGTH
       膝轴 − 股轴 = 159.90 − 70.40 = 89.50 = FEMUR_LENGTH
   （与原项目 hexapod.json 完全一致——真实轴就是关节链。）

   用容器原点当关节的后果：mesh 绕错误的点旋转。膝盖那 10.1mm 的偏差
   在 RViz 里表现为"胫杆与舵机连不紧"——一转就脱开，且**不报任何错**。

   注意股/膝两轴是沿 y 的直线，y 取值不影响旋转；这里取 y=-3.17（与髋轴
   同 y），保证 mesh 平移后恰好落在图纸的装配位置上。

③ **`HorizontalConnector` 等与 `Arm` 是平级容器**，不是 `Arm` 的子对象。
   只导 `Arm` 会漏掉髋的水平连接件。

--------------------------------------------------------------------------
两个容易把几何搞错的点
--------------------------------------------------------------------------
· Draft 克隆（`Part::FeaturePython` + `draftobjects.clone`）的 `.Shape`
  **已含自己的 Placement**——再乘 `getGlobalPlacement()` 会重复应用一次，
  膝舵机就是这么"消失"过（详见 world_shape 的注释）。
· `Slice` / 克隆副本是**派生体**：`ArmHolderBase`、`Slice005`、
  `TopHolder+BottomHolder` 三者几何完全重合。不去重的话体积会翻几倍。
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

# 各连杆的关节（= 舵机输出轴）在图纸全局坐标里的位置（mm），见文件头 ② 的说明
JOINTS = {
    "coxa": (10.10, -3.17, 15.33),
    "femur": (70.40, -3.17, 15.33),
    "tibia": (159.90, -3.17, 15.33),
}

# ---------------------------------------------------------------------------
# 机身
# ---------------------------------------------------------------------------
# 机身是**上下两层**结构，所以必须整体从机身图纸导出，不能只挑几个 STL：
# 图纸里的 Body003(PCB_Connector) 与 pcb_chasis_holder.stl 的 z 中心差 15mm，
# 混用两边的坐标会错位。同一个文档里导，各层天然共享坐标系。
#
# 结构件（y 是板法线方向，即机身的高度）：
#     Body        底板             y -73.59 ~ -53.59   164.0×20.0×180.8
#     Body002     支柱(Standoffs)  y -67.59 ~ -39.59   116.6×28.0×154.8
#     Body001     电池架           y -47.59 ~ -42.59    77.3× 5.0×119.6
#     Body003     PCB 托板         y -39.00 ~ -16.00   164.0×23.0×181.1
#     Body005     上盖             y -16.00 ~  19.00    98.6×35.0×138.2
#     Body004     小凸台           5×5×5 的小件，位于上盖区域
#
# ⚠ **不要导电池 / 电池仓**（踩过的坑）：`Part004`(Battery) 和 `Part001`(BatteryCase)
#   在图纸里**没有装配到位**——实测它们在机身帧里是 z −68 ~ −48mm（底板底面才
#   −41.56mm，也就是吊在底板下方 7~27mm），而且 x 跨度 ±120mm、比机身本身的
#   ±90mm 还宽。导进来的结果是机身底下多出一坨悬空的、比机身还大的几何，
#   比"缺件"更难看。所以电池相关一律不导，等图纸把它们装配好再说。
#
# 同理，树莓派 / LED / 螺钉也不导：它们是电子件，加了只会让 body.stl
# 变大、RViz 变慢。
#
# 舵机也不在这里导（机身图纸里本来就没有舵机实体，18 台全在腿图纸里）：
# 腿上的舵机见下面 EXCLUDE_SUBTREES 的说明。
BODY_DWG = os.path.expanduser(
    "~/Desktop/exist_urdf/hardware/freecad/Hexapod-Chassis.FCStd"
)
BODY_OBJECTS = (
    "Body",        # 底板
    "Body002",     # 支柱
    "Body001",     # 电池架
    "Body003",     # PCB 托板
    "Body005",     # 上盖
    "Body004",     # 上盖上的小凸台
)

# 机身网格的两个精度档 + 面数下限。
# 教训：上盖是薄壁壳（bbox 47 万 mm³、实体只占 6%），在 0.25 的弦高下
# MeshPart 会退化成只剩顶面一小片（实测 25 个三角形），而**不报错**——
# 导出结果看起来"成功了"，RViz 里却是一块板悬在机身上方 30mm。
# 所以这里逐 solid 网格化 + 数面，太少就降精度重试并打印告警。
BODY_LINEAR_DEFLECTION = 0.25
BODY_FINE_DEFLECTION = 0.05
MIN_FACETS_PER_PART = 100

PLATE_CENTER = (0.0, -63.59, 0.0)     # 底板在图纸坐标里的中心
PLATE_HALF_THICK = 10.0               # 底板厚 20mm 的一半
# 髋座（ServoHolder）底面相对**机身帧 z=0**（= 髋轴高度，图纸 z=15.33）的高度：
# 髋座底面在图纸 z = -9.41 → -9.41 - 15.33 = -24.74。
# （旧值 -21.56 是相对 ArmTip 容器原点 z=12.15 量的；关节轴修正后整体降 3.18mm。）
MOUNT_BOTTOM_Z = -24.74

# 机壳自身坐标 → URDF 机身帧（x 前 / y 左 / z 上）：
#     chassis x → body y     （机壳 156 那条边 → 左右）
#     chassis y → body z     （板法线 → 上）
#     chassis z → body x     （机壳 174.78 那条边 → 前后）
# 依据：机壳轮廓沿 ±x 的极值点跨度只有 23.3mm（两个窄凸耳），沿 ±z 有 105.1mm（整条长边）。
# **窄凸耳朝的就是中腿**（中腿正对侧方伸出）→ 所以 chassis x 对应 body y。
BODY_ROT = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


# App 级别的克隆：.Shape 同样已经带上了自己的 Placement
APP_CLONE_TYPES = ("App::Clone", "App::Link", "App::LinkElement")


def is_preplaced(obj):
    """判断 `obj.Shape` 是否**已经包含自己的 Placement**。

    ⚠ 大多数对象的 `Shape` 是**局部坐标**（乘 `getGlobalPlacement()` 才对），
    但**克隆类**不是——它们把自己的 Placement 烘进了 Shape。

    实测：`Clone008`（膝盖上那颗舵机）自己的 Placement 是 (-69.3, -6.35, 0)，
    而它的 `.Shape` 包围盒正好是 x -96.40 ~ -42.20（中心 -69.30，与 Placement 一致）。
    一旦再乘自己的全局 Placement，它就被搬到髋部、与另一颗舵机完全重合，
    去重时被当成副本删掉——**膝盖上的舵机就这么"消失"了**，
    不报任何错，只能靠逐个零件核对才发现。

    判定方式：**有 `Objects` 属性**（被克隆的源对象列表）。图纸里这些是 Draft 克隆
    （`TypeId` 是笼统的 `Part::FeaturePython`，Proxy 才是 `draftobjects.clone.Clone`，
    所以只看 TypeId 认不出来）。而 `Slice`（CompoundFilter）没有 `Objects` 属性，
    它的 Shape 是普通局部坐标，正好被这个判据排除。
    """
    return obj.TypeId in APP_CLONE_TYPES or hasattr(obj, "Objects")


def world_shape(obj, container):
    """取零件在图纸装配坐标系里的几何。

    `container` 是**当前所在容器**（由遍历过程显式传下来，见 collect_world_shapes）。
    ⚠ 不能用 `obj.getParentGroup()` 去查：对这些克隆它返回 **None**
    （虽然它们确实在 `MiddleArm.Group` 里），于是会退化成单位矩阵，
    把零件留在自己的坐标原点——膝盖那颗舵机就是这么落到髋后方的。
    """
    shape = obj.Shape.copy()
    if is_preplaced(obj):
        matrix = container.getGlobalPlacement().toMatrix()   # Shape 已在容器的坐标系里
    else:
        matrix = obj.getGlobalPlacement().toMatrix()
    shape.transformShape(matrix)
    return shape


def xyz(value):
    """坐标可能是 Base.Vector，也可能是 (x,y,z) 元组——FreeCAD 不同路径不一致。"""
    try:
        return value.x, value.y, value.z
    except AttributeError:
        return value[0], value[1], value[2]


# 图纸把髋舵机归进了 `Arm`（髋容器），但**物理上它固定在机身上**：
# 它的输出轴就是 coxa 的旋转轴，本体螺栓在机壳上，所以整台舵机不跟着腿转。
# 烘进 coxa.stl 的话它会跟着腿一起摆——它离轴 11.6mm，中间腿转 90° 时肉眼可见。
# 所以这里排除掉，改由 generate_urdf.py 作为独立 fixed link 挂到 spider_base 下。
#
# 另两台（股 / 膝）留在 femur.stl 里：它们的中心正好在各自的关节轴上，
# 挂哪一节几乎看不出来，没必要拆。
EXCLUDE_SUBTREES = {
    "coxa": ("MG996R",),     # Arm 里的舵机容器
}


def collect_world_shapes(obj, out, container=None, skip=()):
    """把子树里的**最终实体**直接取成"图纸装配坐标系里的几何"，放进 out。

    ⚠ 克隆（Draft Clone）要当**叶子实体**收下，不能跳过：
    图纸里 `MiddleArm` 的两颗舵机（MG996R013 髋 / MG996R014 膝）
    就是克隆，它们的 `.Shape` 是有效实体。跳过 `App::*` 时期股杆上一颗舵机都没有。

    ⚠ 容器的变换要**显式往下传**：
      · 普通零件 → 用对象自己的 `getGlobalPlacement()`
      · 预置位形的克隆 → 用**所在容器**的（它的 Shape 已经含自己的 Placement）
    `App::Part` 会带着自己的变换成为新的容器；`App::DocumentObjectGroup`
    （比如 `GrExplode_Slice`）不改变坐标系，容器保持不变。
    """
    if obj.Name in skip:
        return
    if container is None:
        container = obj

    group = getattr(obj, "Group", None)
    if not group:
        try:
            if obj.Shape.Volume > 0:
                out.append(world_shape(obj, container))
        except Exception:
            pass
        return
    for child in group:
        if child.Name in skip:
            continue
        tid = child.TypeId
        if tid == "App::Part":
            collect_world_shapes(child, out, child, skip)
            continue
        if tid == "App::DocumentObjectGroup":
            collect_world_shapes(child, out, container, skip)
            continue
        # 其余都先试着当实体；拿不到 Shape 才继续往下钻
        try:
            if child.Shape.Volume > 0:
                out.append(world_shape(child, container))
                continue
        except Exception:
            pass
        collect_world_shapes(child, out, container, skip)


def dedup(shapes):
    """去掉重复的实体。

    图纸里同一块几何会以好几种形式重复出现：

    | 例子 | 关系 |
    |---|---|
    | `ArmHolderBase` / `Clone009` / `Slice005` | 完全重合 |
    | `Slice005.1` + `Slice005.0` | 是 `ArmHolderBase` 的**两个切片** |
    | `TopHolder` / `BottomHolder` | 就是那两个切片 |

    ⚠ **判"被包含"不能只看包围盒**。第一版就是这么写的，结果髋杆从 19 件掉到 9 件——
    `ConnectorVertical` 是个 C 形支架，它的**包围盒**套住了旁边的舵机，
    但舵机并不在它的材料里，于是一整颗舵机被误删。
    轴对齐包围盒包含只是几何包含的**必要条件**。

    所以这里分两步：
      ① 体积 + 包围盒完全相等 → 直接去重（真正的副本）
      ② 包围盒被包住的候选 → 再用**布尔求交**确认：`A ∩ B ≈ A` 才说明 A 真的在 B 里

    布尔运算只对少数候选做（包围盒包含是强筛选），不会拖慢多少。
    """
    tol = 1e-6
    entries = []
    for shape in shapes:
        try:
            entries.append((shape.Volume, shape.BoundBox, shape))
        except Exception:
            continue
    entries.sort(key=lambda e: -e[0])

    keep = []
    for vol, box, shape in entries:
        duplicate = False
        for kval, kbox, kshape in keep:
            # ① 完全重合
            if (abs(vol - kval) < 0.01
                    and abs(box.XMin - kbox.XMin) < 0.01 and abs(box.XMax - kbox.XMax) < 0.01
                    and abs(box.YMin - kbox.YMin) < 0.01 and abs(box.YMax - kbox.YMax) < 0.01
                    and abs(box.ZMin - kbox.ZMin) < 0.01 and abs(box.ZMax - kbox.ZMax) < 0.01):
                duplicate = True
                break
            # ② 包围盒被包住 → 布尔确认
            if not (box.XMin >= kbox.XMin - tol and box.XMax <= kbox.XMax + tol
                    and box.YMin >= kbox.YMin - tol and box.YMax <= kbox.YMax + tol
                    and box.ZMin >= kbox.ZMin - tol and box.ZMax <= kbox.ZMax + tol):
                continue
            try:
                common = shape.common(kshape)
                if common.Volume > 0 and abs(common.Volume - vol) <= max(1.0, vol * 0.01):
                    duplicate = True          # A ∩ B ≈ A → A 整个在 B 里
                    break
            except Exception:
                pass                          # 布尔失败就当没被包含（宁可多留，不可误删）
        if not duplicate:
            keep.append((vol, box, shape))
    return [shape for _, _, shape in keep]


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
        shapes = []
        skip = EXCLUDE_SUBTREES.get(label, ())
        for root in roots:
            collect_world_shapes(by_name[root], shapes, skip=skip)
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


def mesh_body_object(obj, deflection):
    """机身单个对象的网格化：**逐 solid 做**，再拼起来。

    为什么要拆开：整个对象的 Shape 一次性传给 MeshPart 时，如果其中某个
    solid 网格化退化，MeshPart 会**静默地少给面**、不报错，事后根本看不出
    是哪个 solid 丢的（上盖就是这样只剩了 25 个面）。拆开做至少能在
    打印里看出每个件各自生成了多少面。
    """
    import numpy as np

    # App::Part 容器本身**没有 Shape 属性**（FreeCAD 1.x 起），要下钻到最终实体；
    # PartDesign::Body 则本身就是成品。
    shapes = []
    if hasattr(obj, "Shape"):
        # 叶子实体：容器就是它自己（两种分支下的变换相同）
        shapes.append(world_shape(obj, obj))
    else:
        collect_world_shapes(obj, shapes)
    shapes = dedup(shapes)

    chunks = []
    for shape in shapes:
        for solid in shape.Solids:
            mesh = MeshPart.meshFromShape(Shape=solid, LinearDeflection=deflection,
                                          AngularDeflection=0.5, Relative=False)
            tris = mesh_to_array(mesh)
            if len(tris):
                chunks.append(tris)
    if not chunks:
        return np.empty((0, 3, 3))
    return np.vstack(chunks)


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
        tris = mesh_body_object(obj, BODY_LINEAR_DEFLECTION)
        if len(tris) < MIN_FACETS_PER_PART:
            # 薄壁壳在粗精度下会退化成极少几个面（上盖踩过），降精度重试。
            retry = mesh_body_object(obj, BODY_FINE_DEFLECTION)
            print(f"  ⚠ {name} 只生成 {len(tris)} 面，"
                  f"降到 {BODY_FINE_DEFLECTION} 重试 → {len(retry)} 面")
            tris = retry
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
