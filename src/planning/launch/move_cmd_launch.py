from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #  启动主车运动指令节点
    car_move_cmd = Node(
        package="planning",
        executable="car_move_cmd",
        name="car_move_cmd",
    )

    #  启动障碍物运动指令节点
    obs_move_cmd = Node(
        package="planning",
        executable="obs_move_cmd",
        name="obs_move_cmd",
    )

    return LaunchDescription([car_move_cmd,obs_move_cmd])
