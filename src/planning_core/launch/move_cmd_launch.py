from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # start ego car move cmd node
    ego_car_move_cmd = Node(
        package="planning_core",
        executable="ego_car_move_cmd",
        name="ego_car_move_cmd"
    )
    # start tp car move cmd node
    tp_move_cmd = Node(
        package="planning_core",
        executable="tp_move_cmd",
        name="tp_move_cmd"
    )

    return LaunchDescription([ego_car_move_cmd, tp_move_cmd])