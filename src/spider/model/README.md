# Spider Upper 舵机

> **当前选型：MG996R × 18**
> 2026-09-28 重写。旧的 MG90S / SG90 记录移到文末「历史」一节，模型文件保留但不代表当前选型。

---

## 选用舵机：MG996R

| 参数 | 厂商标称（参考值） | 说明 |
|---|---|---|
| 尺寸（**本体**，不含耳） | 40.7 × 19.7 × 42.9 mm | 官方口径指的是不含安装耳的本体 |
| 尺寸（**含耳总长**） | ~55 mm | 见下方实测对照 |
| 重量 | ~55 g | 18 个约 **990 g** |
| 堵转扭矩 | ~9.4 kg·cm @4.8V / ~11 kg·cm @6V | 决定整机承重 |
| 空载速度 | ~0.17 ~ 0.19 s / 60° @4.8V | ≈ 320 ~ 350 °/s，**是关节角速度的上限** |
| 齿轮 | 金属 | |
| 工作电压 | 4.8 ~ 7.2 V | |
| 角度范围 | 0 ~ 180° | 与协议 0 ~ 1800 ddeg 一一对应 |

> ⚠ **标称值不同品牌/批次差异不小。** 拿到实物后应核对尺寸、并实测扭矩与速度——
> 它们直接决定机器人的承重能力和步态参数上限。
> 尤其注意「空载速度」：`max_linear_accel_mps2`（速度平滑的加速度上限）目前是估的 0.4 m/s²，
> 真实值要从这个角速度倒推。

### 官方标称 vs 图纸实测

图纸里的 `MG996R` 对象量出来是这样，和官方口径**对得上**：

| | 官方标称 | 图纸实测 | |
|---|---|---|---|
| 本体（不含耳）长 | 40.7 mm | **43.20 mm** | 差 2.5mm，可能是不同批次/含卡扣 |
| 宽（厚度） | 19.7 mm | **20.13 mm** | ✓ |
| 本体高 | 42.9 mm | **42.50 mm** | ✓ |
| **含耳总长** | **~55 mm** | **54.20 mm** | ✓ 关键尺寸吻合 |
| 含输出轴总高 | — | **46.63 mm** | 轴从本体顶面再凸出 4.13mm |

> 所以看到 "40.7mm" 时不要以为这是舵机总长——**装到机械件上占的空间是 54 × 20 × 47mm**。
> 这一点在排布机身上的 18 个舵机时很关键。

### 为什么是 MG996R

**因为机械图纸就是按它设计的**，不是独立做的选型：

- 图纸工程（`~/Desktop/exist_urdf/hardware/`）的 STL 零件上，舵机仓是按 MG996R 开的槽——
  换别的型号装不进去
- 图纸的连杆长度（coxa 60.3 / femur 89.5 / tibia 154.745 mm）
  和六个腿座的位置，都是围绕 MG996R 的尺寸确定的

这是一个**从机械反推电气**的结果。

### 整机影响

| 项 | 数值 |
|---|---|
| 数量 | 18（6 条腿 × 3 个关节） |
| 舵机总重 | ~990 g |
| 整机估重 | ~1.5 ~ 2 kg（加电池、树莓派 5、PCB、打印框架） |
| 供电 | 18 路可能同时动作，**峰值电流不能按单个舵机估** |

---

## 模型文件

| 文件 | 说明 | 状态 |
|---|---|---|
| **`body.stl`** | **机身**（上下两层共 6 个结构件），已在 URDF 机身帧、单位米 | ✅ URDF 在用 |
| **`coxa.stl`** | **髋连杆**，原点在髋关节、单位米 | ✅ URDF 在用 |
| **`femur.stl`** | **股连杆**，原点在股关节、单位米 | ✅ URDF 在用 |
| **`tibia.stl`** | **胫连杆**，原点在膝关节、单位米 | ✅ URDF 在用 |
| `mg996r.stl` | MG996R 完整装配体（本体+安装耳+减振垫+线缆+输出轴，11 个零件合一） | 参考 |
| `mg996r.step` | 同上，**STEP 格式**——要改机械件、做干涉检查时用这个 | 参考 |
| `mg90s.stl` | MG90S 的 3D 模型 | ⚠ **已废弃** |
| `sg90.stl` | SG90 的 3D 模型 | ⚠ **已废弃** |
| `servo_lib/` | 开源 OpenSCAD 参数化模型库 | 参考用，**不含 MG996R** |
| `generate_servo_stl.scad` | STL 生成脚本 | 仅对旧选型有效 |

> ⚠ `mg90s.stl` / `sg90.stl` 是**旧选型的模型**，和现在用的 MG996R 尺寸差一倍以上。
> 留着只是为了追溯，**设计零件时不要用错**。
>
> ⚠ **`body/coxa/femur/tibia.stl` 和 `mg996r.stl` 的坐标系不一样**：前者是给 URDF 用的
> （已在各自连杆帧、单位米），后者是零件本身（图纸坐标、单位毫米）。别混用。

### 连杆 / 机身 mesh 是怎么来的（URDF 用）

四个文件都由图纸导出，**原点在各自连杆的关节上、单位是米**，所以 URDF 里
`<visual>` / `<collision>` **不写 `<origin>`、不写 `scale`** —— 多写任何一处，
RViz 里就会表现为"mesh 飘在关节外面"，**且不报错**。

```
body.stl    机身 6 层：底板 Body + 支柱 Body002 + 电池架 Body001
                      + PCB 托板 Body003 + 上盖 Body005 + Body004
coxa.stl    髋：Arm（**不含舵机**）+ HorizontalConnector + MotorConnector
                + ArmCap + BaseHolder
femur.stl   股：MiddleArm（**含 2 台舵机**：髋位 MG996R013 + 膝位 MG996R014）
tibia.stl   胫：ArmTip
mg996r.stl  单台舵机，被 6 个髋舵机 link 复用
```

### 18 台舵机的分布

| 舵机 | 固定在哪一节 | 在哪 |
|---|---|---|
| 髋 ×6 | **机身** | 独立 link（`{leg}_coxa_servo_link` + `mg996r.stl`） |
| 股 ×6 | 髋 | 烘在 `femur.stl` 里（中心正好在股关节轴上，挂哪节几乎看不出） |
| 膝 ×6 | 股 | 烘在 `femur.stl` 里（同上） |

**髋舵机为什么要单独放**：它的本体螺栓在机壳上、输出轴就是 coxa 的旋转轴，
所以整台舵机**不跟着腿转**。烘进 `coxa.stl` 的话它会跟着摆——它离轴 11.6mm，
中腿转 90° 时肉眼可见。所以 `export_cad_meshes.py` 把 `Arm` 里的舵机容器排除了
（见 `EXCLUDE_SUBTREES`）。

> ⚠ **曾经误判"图纸里只有 1 台舵机"**：`MiddleArm` 的 `MG996R013/014` 是
> **Draft 克隆**（`TypeId` 是笼统的 `Part::FeaturePython`，Proxy 才是
> `draftobjects.clone.Clone`），它的 `.Shape` **已经包含自己的 Placement**。
> 再乘 `getGlobalPlacement()` 会重复应用一次 → 两台重叠成一台 → 被去重删掉。
> 结果就是"膝盖舵机消失"，而且**不报任何错**。
> 判据：克隆有 `Objects` 属性（被克隆的源对象列表），`Slice`（CompoundFilter）没有。

> ⚠ **机身必须整体从同一份图纸导出**，不能挑几个 STL 拼：
> `pcb_chasis_holder.stl` 与图纸里的 `Body003` 的 **z 中心差 15mm**，
> 混用两边的坐标会让上层板和底板错位。
>
> 另外机身是**上下两层**的——只导底板（`chassis.stl`）会得到"上半部分是空的"，
> 实物照片上看起来是单层，是因为电子件（PCB、电池）直接压在底板上，
> 而 CAD 里它们是有独立托板和支柱的。

**为什么需要 FreeCAD**：`.FCStd` 里的 `Placement` 是逐层累乘的（`App::Part` 容器自己也带旋转），
手写 XML 解析极容易把旋转方向算反（本项目就踩过：y 的符号错了，导致零件"横"着，
白绕了一大圈才发现）。用官方 API 的 `getGlobalPlacement()` 就没有这个风险。

```bash
conda create -y -n freecad -c conda-forge freecad   # apt 里是 0.19，打不开 1.x 格式
```

**关键事实（决定了脚本怎么写）**：

| 事实 | 说明 |
|---|---|
| 图纸**是装好的腿** | 全局坐标下 `coxa`/`femur`/`tibia` 三段在关节处自然衔接，是一整条腿 |
| **关节轴 = 舵机输出轴** | 不是容器原点。用 FreeCAD 量舵机输出轴（沿轴线的最大圆柱，R10）得真实旋转轴：髋 (10.10, -3.17, ·)、股 (70.40, ·, 15.33)、膝 (159.90, ·, 15.33)。两个**精确到 0.01mm** 的吻合可互证：`70.40−10.10=60.30=COXA_LENGTH`、`159.90−70.40=89.50=FEMUR_LENGTH` |
| ⚠ 容器原点 ≠ 关节 | `ArmTip` 容器原点 (149.80, -14.77, 12.15) 只是建模参考点。用它当膝关节会让 mesh 绕错误的点转——RViz 里胫杆与舵机"连不紧"，一转就脱开且**不报错** |
| `HorizontalConnector` 与 `Arm` **平级** | 不是 `Arm` 的子对象，只导 `Arm` 会漏件 |
| **Draft 克隆的 `.Shape` 已含自己的 Placement** | 见上面 18 台舵机那节——再乘 `getGlobalPlacement()` 会让膝盖舵机消失 |
| `Slice` 是**派生体** | `ArmHolderBase` / `Slice005` / `TopHolder+BottomHolder` 三者几何重合，必须去重 |
| 去重不能只看包围盒 | `ConnectorVertical` 是 C 形支架，包围盒套住了旁边的舵机但材料没套住——一整台舵机被误删过。要用布尔求交确认 |
| `App::Part` 带 `Extensions="True"` | 正则匹配 `<Object name="X">` 会漏掉它们——而装配容器恰好都带这个属性 |
| 机身朝向要**量**出来 | 机壳沿 ±x 的极值点跨度只有 23.3mm（窄凸耳，朝中腿），沿 ±z 有 105.1mm（长边） |
| 机壳高度 | 髋座（`coxa_mount`）底面在**髋轴高度**（图纸 z=15.33）下方 **24.74mm**，机壳顶面贴在那里 |
| 髋舵机**不能烘进 `coxa.stl`** | 它固定在机身上、输出轴就是 coxa 的旋转轴；MG996R 的轴偏离本体中心 10.1mm，跟着转会看得出来 |
| `<origin>` 写在 visual 还是 joint 上**不一样** | 写进 joint 的话偏移会跟着 yaw 再转一次，中腿的舵机被甩到错的地方 |

**重新生成**（改机械件后）：

```bash
cd src/spider
~/miniconda3/envs/freecad/bin/freecadcmd \
    -c "exec(open('scripts/export_cad_meshes.py').read())"
```

脚本是 `scripts/export_cad_meshes.py`，里面有全部坐标依据的注释。
核心逻辑是：**对每个连杆取"相对所在容器原点"的 Placement（几何形状），再整体减去
该连杆的关节（= 舵机输出轴）全局坐标，最后 ×0.001 换算成米**。背景见
`工程现状总结.md` 的 5.17 节。



从图纸 `~/Desktop/exist_urdf/hardware/freecad/Hexapod-Leg.FCStd` 里导出的。
那个文件里有一个名为 **`MG996R`** 的 `App::Part` 容器，装着 11 个零件：

| 零件 | 尺寸 (mm) | 是什么 |
|---|---|---|
| `Part__Feature` | 43.20 × 20.00 × 42.50 | 本体 |
| `Part__Feature001` ~ `004` | ⌀3.25 × 1.51 | 底面 4 个减振垫 |
| `Part__Feature005` ~ `007` | 3.00 × 1.50 × 1.50 | 出线的 3 段 |
| `Part__Feature008` / `009` | 7.00 × 19.00 × 4.50 | 左右安装耳（各 1 个） |
| `Part__Feature010` | ⌀5.90 × 5.06 | 输出轴 |

**导出过程（可复现）**：`.FCStd` 本质是个 zip，里面的 `*.brp` 是 **ASCII 版 BRep 文本**
（不是二进制），所以不需要装 FreeCAD，用 OpenCASCADE 的 Python 绑定就能读：

```bash
pip install cadquery-ocp                    # 提供 OCP 绑定（约 130MB）
# 国内网络建议加： -i https://pypi.tuna.tsinghua.edu.cn/simple
```

```python
import zipfile, os
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Shape, TopoDS_Compound
from OCP.BRepTools import BRepTools
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.STEPControl import STEPControl_Writer, STEPControl_StepModelType

os.makedirs('/tmp/fc', exist_ok=True); os.chdir('/tmp/fc')
zipfile.ZipFile('/home/liujiarui/Desktop/exist_urdf/hardware/freecad/Hexapod-Leg.FCStd').extractall('.')

members = ['Part__Feature'] + [f'Part__Feature{i:03d}' for i in range(1, 11)]
builder = BRep_Builder(); comp = TopoDS_Compound(); builder.MakeCompound(comp)
for name in members:
    s = TopoDS_Shape(); BRepTools.Read_s(s, f'{name}.Shape.brp', builder)
    if not s.IsNull(): builder.Add(comp, s)

BRepMesh_IncrementalMesh(comp, 0.02, False, 0.5, True)   # 0.02mm 网格精度
StlAPI_Writer().Write(comp, 'mg996r.stl')

w = STEPControl_Writer(); w.Transfer(comp, STEPControl_StepModelType.STEPControl_AsIs)
w.Write('mg996r.step')
```

> 想导出**别的零件**（比如整条腿的装配体）时，把 `members` 换成对应的对象名即可。
> 对象名可以在 `Document.xml` 里查：`grep -o 'name="[^"]*"' Document.xml | sort -u`。

### 重新生成模型（仅对旧选型有效）

```bash
cd src/spider/model
openscad -o mg90s.stl -D 'model_name="MG90S (Clone)"' generate_servo_stl.scad
openscad -o sg90.stl  -D 'model_name="SG90 (Clone)"'  generate_servo_stl.scad
```

---

## 历史：为什么放弃了 MG90S

2026-09 之前本工程选的是 MG90S，理由是便宜、够小、金属齿。当时的判断是：

> 建议腿的关键承力关节（femur / tibia）用 MG90S，coxa 关节可考虑 SG90 降低成本。

**改变的原因就是拿到了机械图纸**——图纸按 MG996R 设计。两个型号差着一辈：

| | MG90S | **MG996R** |
|---|---|---|
| 重量 | 13.4 g | **~55 g** |
| 扭矩 | ~1.8 kg·cm | **~9.4 ~ 11 kg·cm** |
| 尺寸 | 22.9 × 12.4 × 32.4 mm | **40.7 × 19.7 × 42.9 mm** |
| 价格 | ~¥15-20 | ~¥20-30 |

代价很实在：**舵机总重从 240 g 涨到 990 g**，结构强度和供电都要重新考虑。
换来的是扭矩提升 5 倍多——对 304.5 mm 展长的腿来说，这是必需的。

### MG90S 的实测几何尺寸（存档）

```
Body width:  22.92 mm    (主体宽)
Body length: 12.38 mm    (主体长)
Aft height:  18.31 mm    (后段高)
Fore height:  7.62 mm    (前段高)
Wing height:  2.53 mm    (安装耳厚度)
Wing width:  32.38 mm    (安装耳总宽)
Axle diameter: 4.70 mm   (输出轴径)
Axle height:   4.00 mm   (输出轴高)
Axle offset:   6.19 mm   (输出轴偏移)
```

### SG90（更早的备选，已废弃）

塑料齿，~9 g，~1.2 kg·cm。在足式机器人关节中容易扫齿，当时就只建议用于轻载验证。

---

## 相关文档

- 整机配置与图纸来源：`工程现状总结.md` 第 13 节
- 舵机标定（`offset` / `direction`）：`servo_map.yaml` + 决策 D3
- 下位机侧的标定兜底：`工程现状总结.md` 第 11.4 节
