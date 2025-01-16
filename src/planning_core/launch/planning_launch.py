from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # set path
    planning_path = get_package_share_directory("planning_core")
    
    # ego vehicle model
    car_path = os.path.join(planning_path, "urdf/ego_car_model", "car.xacro")

    car_para = ParameterValue(Command(["xacro ", car_path]))

    car_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="car_state_pub",
        output="screen",
        parameters=[{"robot_description": car_para}]
    )


    # joint state publisher with GUI
    car_joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="car_joint_state_pub"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2"
    )

    # launch planning node
    planning_node = Node(
        package="planning_core",
        executable="planning_node",
        name="planning_node"
    )

    # node group
    car_main = GroupAction(
        actions=[
            PushRosNamespace("car"),
            car_state_pub,
            car_joint_state_pub_gui
        ]
    )

    planning = GroupAction(
        actions=[
            PushRosNamespace("planning_core"),
            planning_node
        ]
    )

    return LaunchDescription([car_main, rviz2, planning])