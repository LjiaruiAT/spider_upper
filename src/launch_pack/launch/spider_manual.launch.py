"""手动调试模式：绕过数学链，直接指定 18 路舵机角 —— 单路 / 单腿调试入口。

启动内容：
    manual_servo_node   读 spider/config/default_pose.yaml 作为初值（当前全 900），
                        提供 /spider/manual_servo/set_angles 服务，1Hz 重发当前值
    robot_driver_node   把 18 路打包成 42 字节协议帧；
                        fake_send:=true（默认）只打印；fake_send:=false 真发串口

**不启动 leg_calc**：manual_servo_node 和 leg_calc 会往同一个
/spider/servo_target 发布，两个发布者同时存在时 driver 收到的是两路帧的交错，
真机上就是两个控制源打架。要跑正常控制请用 spider_minimal.launch.py。

用法：
    # ① 打印模式（不碰设备，先确认节点与话题正常）
    ros2 launch launch_pack spider_manual.launch.py

    # ② 真发送（接下位机 + 舵机）—— 设备打不开会启动失败并打印原因
    #    设备默认用固定名 /dev/spider_stm32（需 udev 规则，见 调试方法.md §2.1），故不用传 device_name；
    #    没装规则时改用实际设备名，如 device_name:=/dev/ttyACM0（别写死编号，J-Link 占位会变）
    ros2 launch launch_pack spider_manual.launch.py \\
        fake_send:=false

    # 单路调试（下标 0 = lf coxa = PE9）：转到 120.0°
    ros2 service call /spider/manual_servo/set_angles \\
        robot_interfaces/srv/SetServoAngles "{channels: [0], angles: [1200]}"

    # 单腿调试：一次设一条腿的 3 路（lf 的 coxa / femur / tibia）
    ros2 service call /spider/manual_servo/set_angles \\
        robot_interfaces/srv/SetServoAngles \\
        "{channels: [0, 1, 2], angles: [900, 800, 900]}"

    # 第一次上电从 900（中位）附近小幅度试起；下标 ↔ 腿/关节 ↔ 引脚全表见
    # 舵机序号与装配.md
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    driver_params = os.path.join(
        get_package_share_directory('robot_driver'), 'config', 'driver.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fake_send',
            default_value='true',
            description='true=只打印协议帧；false=真发串口（需接硬件）',
        ),
        DeclareLaunchArgument(
            'device_name',
            default_value='/dev/spider_stm32',
            description='真实串口设备；fake_send:=false 时使用',
        ),
        Node(
            package='spider_task',
            executable='manual_servo_node',
            name='manual_servo_node',
            output='screen',
        ),
        Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver_node',
            output='screen',
            parameters=[
                driver_params,
                {
                    'device_name': ParameterValue(
                        LaunchConfiguration('device_name'), value_type=str),
                    'fake_send': ParameterValue(
                        LaunchConfiguration('fake_send'), value_type=bool),
                },
            ],
        ),
    ])


