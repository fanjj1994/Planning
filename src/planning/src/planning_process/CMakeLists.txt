cmake_minimum_required(VERSION 3.8)
project(planning_process)

#设置依赖
set(DEPENDS
    "rclcpp"
    "nav_msgs"
    "base_msgs"
    "tf2"
    "tf2_ros"
)

#添加节点
add_executable(planning_process
    ${CMAKE_CURRENT_SOURCE_DIR}/planning_process.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/planning_process_main.cpp
)

set(target
    planning_process
)

#添加依赖
ament_target_dependencies(${target}
    ${DEPENDS}
)

target_link_libraries(${target}
    config_reader
    decision_center
    vehicle_info
    reference_line
    local_planner
)

#安装动态库、节点和相关目录
install(TARGETS
    ${target}
    DESTINATION lib/${PLANNING_PACKAGE_NAME})