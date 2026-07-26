# Spider Upper 舵机模型

> 舵机型号来自 GitHub [GetBurning/servo_lib](https://github.com/GetBurning/servo_lib) 开源参数化模型库。

---

## 选用舵机：MG90S (Clone)

| 参数 | 值 |
|------|------|
| 价格 | ~¥15-20 |
| 重量 | 13.4g |
| 齿轮 | 金属齿 |
| 扭矩 | ~1.8 kg·cm (4.8V) / ~2.2 kg·cm (6.0V) |
| 角度范围 | 0~180° |
| 工作电压 | 4.8V ~ 6.0V |
| 尺寸 (宽×长×总高) | 22.9 × 12.4 × 32.4 mm |

### 关键几何尺寸（来自实测）

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

---

## 备选舵机：SG90 (Clone)

| 参数 | 值 |
|------|------|
| 价格 | ~¥5-8 |
| 重量 | 9g |
| 齿轮 | 塑料齿 |
| 扭矩 | ~1.2 kg·cm (4.8V) |

> SG90 更便宜但塑料齿在足式机器人关节中容易扫齿，建议仅用于轻载验证。

---

## 模型文件

| 文件 | 说明 |
|------|------|
| `mg90s.stl` | MG90S 3D 模型（二进制 STL） |
| `sg90.stl` | SG90 3D 模型（二进制 STL） |
| `servo_lib/` | 开源 OpenSCAD 参数化模型库 |
| `generate_servo_stl.scad` | STL 生成脚本 |

### 重新生成模型

```bash
cd src/spider/model
openscad -o mg90s.stl -D 'model_name="MG90S (Clone)"' generate_servo_stl.scad
openscad -o sg90.stl -D 'model_name="SG90 (Clone)"' generate_servo_stl.scad
```

---

## 六足用量

- 每条腿 3 个关节（coxa / femur / tibia）
- 6 条腿 × 3 = **18 个舵机**
- 总成本（MG90S）：18 × ¥15-20 ≈ **¥270-360**
- 总成本（SG90）：18 × ¥5-8 ≈ **¥90-144**

> 建议腿的关键承力关节（femur / tibia）用 MG90S，coxa 关节可考虑 SG90 降低成本。
