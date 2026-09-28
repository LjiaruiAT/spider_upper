"""显示模式：把机器人画在 RViz 里。

启动内容：
    generate_urdf.py     从 leg_params.yaml 生成 URDF
    robot_state_publisher  由 URDF + /joint_states 算出 TF 树
    leg_calc_node          发布 /joint_states 与 odom→spider_base 的 TF
    spider_task_node       接收 /spider/cmd_vel，否则发不出速度命令
    rviz2                  显示

**不启动 robot_driver**：显示模式不产生真数据，驱动节点没有意义。
要看真实链路请用 `spider_minimal.launch.py`。

用法：
    ros2 launch launch_pack spider_display.launch.py
然后在另一个终端发速度命令，RViz 里就能看到腿动：
    ros2 topic pub -r 20 /spider/cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.2}, angular: {z: 0.0}}'

----------------------------------------------------------------------------
rviz_config 参数：选"从哪个视角看"
----------------------------------------------------------------------------
本文件是显示模式的**唯一实现**，两个视角只是换一份 RViz 配置：

    spider.rviz        固定坐标系 spider_base —— 机身钉在原点，看腿在动
    spider_walk.rviz   固定坐标系 odom       —— 看机器人真的在地面上走

`spider_walk.launch.py` 就是这个文件 + `rviz_config:=spider_walk.rviz`。
刻意做成"一个实现 + 参数"，而不是两份几乎相同的 launch——两份的话以后改
启动内容就得记得改两处，早晚会漏。
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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

    # 只写文件名（相对 launch_pack/rviz/），不写绝对路径：
    # 这个参数就是"选个视角"，从命令行写 `rviz_config:=spider_walk.rviz` 最省事，
    # 让调用方去拼 get_package_share_directory() 反而更容易写错。
    #
    # ⚠ 必须用 PathJoinSubstitution，不能图省事写 os.path.join：
    # LaunchConfiguration 是**延迟求值的替换对象**，不是字符串，
    # os.path.join 会直接抛 "join() argument must be str, bytes, or os.PathLike"，
    # 而且这个错发生在 launch 解析阶段——表现为"launch 直接没起来"，
    # 不是某个节点报错，不太好往这上面想。
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('launch_pack'),
        'rviz',
        LaunchConfiguration('rviz_config'),
    ])

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
        # 见文件开头的说明：两个视角的差别只是固定坐标系。
        DeclareLaunchArgument(
            'rviz_config',
            default_value='spider.rviz',
            description='launch_pack/rviz/ 下的配置文件名；'
                        'spider.rviz = 固定坐标系 spider_base（看腿动），'
                        'spider_walk.rviz = 固定坐标系 odom（看机器人走）',
        ),
        # 步态选择：tripod / ripple / wave（默认 tripod）。
        # 只支持启动时选择（运行中切换会让相位映射突变，见 leg_calc 的说明）。
        # 例：ros2 launch launch_pack spider_walk.launch.py gait_pattern:=wave
        DeclareLaunchArgument(
            'gait_pattern',
            default_value='tripod',
            description='步态模式：tripod / ripple / wave；透传给 leg_calc_node',
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
            parameters=[{'gait_pattern': LaunchConfiguration('gait_pattern')}],
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
