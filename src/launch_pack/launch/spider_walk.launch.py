"""行走视角：在 RViz 里看机器人**在地面上真的走**。

和 `spider_display.launch.py` 的唯一区别是 RViz 的固定坐标系：

    spider_display   Fixed Frame = spider_base
                     机身钉在世界原点（因为 spider_base 是 URDF 的根，
                     **按定义它永远在原点**）。你看到的是六条腿在原地划水——
                     适合检查"每条腿的相位和关节角对不对"。

    spider_walk      Fixed Frame = odom
                     机身会随命令平移、转向，**支撑足在世界系里站住不动**。
                     适合检查"走路看起来对不对"。

----------------------------------------------------------------------------
本文件只有这几行，因为显示模式在 `spider_display.launch.py` 里实现，
两个视角只是换一份 RViz 配置。刻意做成"一个实现 + 参数"，而不是两份几乎
相同的 launch——两份的话以后改启动内容就得记得改两处，早晚会漏。

用法：
    ros2 launch launch_pack spider_walk.launch.py

然后在另一个终端发速度命令（做得到这一步说明环境已经配好了）：
    ros2 topic pub -r 20 /spider/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'

想看转弯（半径 = v / w，能明显看出走的是圆弧）：



⚠ odom 是**按命令速度推算**的，不是传感器测出来的（见 leg_calc 的
   odometry_integrator.hpp）。真机上脚打滑时它会偏，所以只当可视化看，
   别拿去接导航。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    display_launch = os.path.join(
        get_package_share_directory('launch_pack'),
        'launch',
        'spider_display.launch.py',
    )

    return LaunchDescription([
        # 透传给 spider_display：这里声明一次，命令行才能传
        # `gait_pattern:=wave` / `use_rviz:=false`。
        DeclareLaunchArgument(
            'gait_pattern',
            default_value='tripod',
            description='步态模式：tripod / ripple / wave；透传给 leg_calc_node',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动 RViz；false 时只启动数据与 TF',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch),
            launch_arguments={
                'rviz_config': 'spider_walk.rviz',
                'gait_pattern': LaunchConfiguration('gait_pattern'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),
    ])
