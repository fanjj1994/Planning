from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 路径设置
    # 　workspace/install/planning/share/planning目录路径
    planning_path = get_package_share_directory("planning")

    # 　车辆模型位置
    car_path = os.path.join(planning_path, "urdf/main_car", "car.xacro")
    obs_car_path = os.path.join(planning_path, "urdf/obs_car", "obs_car.xacro")

    # 　rviz配置文件位置
    rviz_conf_path = os.path.join(planning_path, "rviz", "planning.rviz")

    # xacro命令行指令, 封装成参数, 用于启动urdf文件, 注意后面要加空格
    car_para = ParameterValue(Command(["xacro ", car_path]))
    obs_car_para = ParameterValue(Command(["xacro ", obs_car_path]))

    # 启动robot_state_publisher节点, 以参数的方式加载urdf文件内容
    car_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="car_state_pub",
        output="screen",
        parameters=[{"robot_description": car_para}],
    )
    obs_car_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="obs_car_state_pub",
        output="screen",
        parameters=[{"robot_description": obs_car_para}],
    )

    # 启动joint_state_publisher节点
    car_joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="car_joint_state_pub",
    )
    obs_car_joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="obs_car_joint_state_pub",
    )

    # #手动控制关节运动的带gui界面的节点
    # car_joint_state_pub_gui = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="car_joint_state_pub",
    # )

    # 启动rviz2节点, 并加载配置文件
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_conf_path],
    )

    # #启动绘图节点
    # data_plot = Node(
    #     package = "data_plot",
    #     executable = "data_plot",
    #     name ="data_plot",
    # )
    
    # 启动地图服务器节点
    pnc_map_server = Node(
        package="planning",
        executable="pnc_map_server",
        name="pnc_map_server",
    )
        

    # 启动全局路径服务器节点
    global_path_server = Node(
        package="planning",
        executable="global_path_server",
        name="global_path_server",
    )

    # 启动规划节点
    planning_process = Node(
        package="planning",
        executable="planning_process",
        name="planning_process",
    )

    # 节点分组
    car_main = GroupAction(
        actions=[
            PushRosNamespace("car"),
            car_state_pub,
            car_joint_state_pub,
        ]
    )
    obs_car = GroupAction(
        actions=[
            PushRosNamespace("obs_car"),
            obs_car_state_pub,
            obs_car_joint_state_pub,
        ]
    )

    planning = GroupAction(
        actions=[
            PushRosNamespace("planning"),
            planning_process,
            pnc_map_server,
            global_path_server
        ]
    )

    return LaunchDescription(
        [
            car_main,
            obs_car,
            rviz2,
           # data_plot,  
            planning,
        ]
    )
