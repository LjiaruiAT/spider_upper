import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # robot_driver 的运行参数（设备名 / 波特率 / fake_send / readback_enabled）
    # 从 config/driver.yaml 加载——那是 ROS 参数文件格式（顶层是节点名）。
    # 用参数文件而不是让节点自己解析：命令行可以只覆盖单个值，例如
    #   ros2 run robot_driver robot_driver_node --ros-args -p device_name:=/dev/ttyUSB0
    driver_params = os.path.join(
        get_package_share_directory('robot_driver'), 'config', 'driver.yaml')

    return LaunchDescription([
        # 步态选择：tripod / ripple / wave（默认 tripod）。
        DeclareLaunchArgument(
            'gait_pattern',
            default_value='tripod',
            description='步态模式：tripod / ripple / wave；透传给 leg_calc_node',
        ),
        Node(
            package='spider_task',
            executable='spider_task_node',
            name='spider_task_node',
            output='screen',
        ),
        Node(
            package='leg_calc',
            executable='leg_calc_node',
            name='leg_calc_node',
            output='screen',
            parameters=[{'gait_pattern': LaunchConfiguration('gait_pattern')}],
        ),
        Node(
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver_node',
            output='screen',
            parameters=[driver_params],
        ),
    ])
