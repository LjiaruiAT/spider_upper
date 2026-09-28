"""显示模式：把机器人画在 RViz 里。

启动内容：
    generate_urdf.py     从 leg_params.yaml 生成 URDF
    robot_state_publisher  由 URDF + /joint_states 算出 TF 树
    leg_calc_node          发布 /joint_states（内容就是 IK 解出的关节角）
    spider_task_node       接收 /spider/cmd_vel，否则发不出速度命令
    rviz2                  显示

**不启动 robot_driver**：显示模式不产生真数据，驱动节点没有意义。
要看真实链路请用 `spider_minimal.launch.py`。

用法：
    ros2 launch launch_pack spider_display.launch.py
然后在另一个终端发速度命令，RViz 里就能看到腿动：
    ros2 topic pub -r 20 /spider/cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.2}, angular: {z: 0.0}}'
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_urdf_description():
    """调用 spider 包的脚本生成 URDF，返回字符串。

    刻意在 **launch 时生成**，而不是读一个预先提交的 .urdf 文件：
    这样 leg_params.yaml 永远是唯一真相，不存在"改了配置忘了重新生成 URDF"
    ——那种错误不会报错，只会在 RViz 里把腿画在错的位置。
    """
    spider_share = get_package_share_directory('spider')
    script = os.path.join(spider_share, 'scripts', 'generate_urdf.py')
    leg_params = os.path.join(spider_share, 'config', 'leg_params.yaml')
    return subprocess.check_output(
        ['python3', script, '--leg-params', leg_params],
        text=True,
    )


def generate_launch_description():
    robot_description = generate_urdf_description()
    rviz_config = os.path.join(
        get_package_share_directory('launch_pack'), 'rviz', 'spider.rviz'
    )

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        # use_rviz:=false 时不启动 RViz，只跑数据和 TF——
        # 便于脚本化验证（'RViz 里的腿没动' 这类问题，先确认 topic 对不对），
        # 也方便只想用命令行看数据的场合。
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz；false 时只启动数据与 TF',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
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
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
        ),
    ])
