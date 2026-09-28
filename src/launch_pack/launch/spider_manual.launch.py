"""手动调试模式：绕过数学链，直接指定 18 路舵机角。

启动内容：
    manual_servo_node   读 spider/config/default_pose.yaml 作为初值（当前全 900），
                        提供 /spider/manual_servo/set_angles 服务，1Hz 重发当前值
    robot_driver_node   把 18 路打包成 42 字节协议帧（当前 fake-send：只打印）

**不启动 leg_calc**：manual_servo_node 和 leg_calc 会往同一个
/spider/servo_target 发布，两个发布者同时存在时 driver 收到的是两路帧的交错，
真机上就是两个控制源打架。要跑正常控制请用 spider_minimal.launch.py。

用法：
    ros2 launch launch_pack spider_manual.launch.py

    # 启动即发中位（default_pose.yaml 是全 900 = 90.0°）
    # 单腿调试：一次设一条腿的 3 路（例：lf 的 coxa / femur / tibia）
    ros2 service call /spider/manual_servo/set_angles \\
        robot_interfaces/srv/SetServoAngles \\
        "{channels: [0, 1, 2], angles: [900, 800, 900]}"

    # 只动 1 号通道（lf femur）到 80.0°
    ros2 service call /spider/manual_servo/set_angles \\
        robot_interfaces/srv/SetServoAngles "{channels: [1], angles: [800]}"

    # 起点仍是保守值：900 附近是舵机中位，第一次上电先只做小幅度测试。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    driver_params = os.path.join(
        get_package_share_directory('robot_driver'), 'config', 'driver.yaml')

    return LaunchDescription([
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
            parameters=[driver_params],
        ),
    ])
