#!/bin/bash

# 打开第一个terminal运行第一个命令
gnome-terminal -- ros2 launch planning_core planning_launch.py &

sleep 2

# 打开第二个terminal运行第二个命令
gnome-terminal -- ros2 launch planning_core move_cmd_launch.py &

# 等待所有terminal都退出后再关闭
wait
