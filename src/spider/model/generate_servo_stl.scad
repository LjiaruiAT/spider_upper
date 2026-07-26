// 从 servo_lib 生成 MG90S 和 SG90 舵机 STL 模型
// 用法：openscad -o mg90s.stl -D 'model="MG90S (Clone)"' generate_servo_stl.scad

include <servo_lib/servo_lib.scad>

// 默认生成 MG90S，可通过 -D 参数覆盖
model_name = "MG90S (Clone)";

ServoLib_GenerateServo(model_name);
