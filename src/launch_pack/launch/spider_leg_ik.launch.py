"""单腿 IK 控制：输入腿局部坐标 (x,y,z)，让指定腿的足端走到那个点。

启动内容：
    leg_ik_node       读 leg_params.yaml（腿长 / 腿座 / 限位），IK 解算后
                      只更新目标腿的 3 路；20ms 定时器按 1 秒平滑插值发布
    robot_driver_node 打包 42 字节协议帧；fake_send:=false 时真发串口

**不启动 leg_calc**：两者都往 /spider/servo_target 发布，同时跑会互相覆盖。

用法：
    # ① 打印模式（不碰设备）
    ros2 launch launch_pack spider_leg_ik.launch.py

    # ② 真发送（接下位机 + 舵机）
    #    设备默认用固定名 /dev/spider_stm32（需 udev 规则，见 调试方法.md §2.1），故不用传 device_name；
    #    没装规则时改用实际设备名，如 device_name:=/dev/ttyACM0（别写死编号，J-Link 占位会变）
    ros2 launch launch_pack spider_leg_ik.launch.py \\
        fake_send:=false

    # 让 lf 腿的足端走到腿局部系的 (121.8, 0, -120)：
    # 即这条腿的足端比站姿往下伸 28.6mm（leg_ik 只动这一条腿，机身不会整体下降）
    ros2 service call /spider/leg_ik/move_to robot_interfaces/srv/LegMoveTo \\
        "{leg: 'lf', x: 121.8, y: 0.0, z: -120.0}"

    # 回站姿
    ros2 service call /spider/leg_ik/move_to robot_interfaces/srv/LegMoveTo \\
        "{leg: 'lf', x: 121.8, y: 0.0, z: -91.4}"
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
            executable='leg_ik_node',
            name='leg_ik_node',
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
