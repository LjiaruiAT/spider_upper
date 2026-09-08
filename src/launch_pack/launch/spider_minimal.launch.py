from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
            package='robot_driver',
            executable='robot_driver_node',
            name='robot_driver_node',
            output='screen',
        ),
    ])
