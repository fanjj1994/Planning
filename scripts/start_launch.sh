#!/bin/bash

# 存储后台进程的PID，便于清理
pids=()

# 信号处理函数：确保所有子进程都被正确终止
cleanup() {
    echo "Shutting down all processes..."
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null
        fi
    done
    # 给子进程时间优雅关闭
    sleep 1
    # 强制杀死仍在运行的进程
    for pid in "${pids[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null
        fi
    done
}

# 设置信号处理器
trap cleanup SIGINT SIGTERM

# 方案1：打开新terminal窗口（原始方式）
gnome-terminal -- bash -c "ros2 launch planning_core planning_launch.py; bash" &
pids+=($!)

sleep 2

# 打开第二个terminal运行第二个命令
gnome-terminal -- bash -c "ros2 launch planning_core move_cmd_launch.py; bash" &
pids+=($!)

# 等待所有进程退出
wait