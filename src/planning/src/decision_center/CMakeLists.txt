cmake_minimum_required(VERSION 3.8)
project(planning)

#设置依赖
set(DEPENDS
    "rclcpp"
)

#添加动态库
add_library(decision_center SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/decision_center.cpp
)

set(target
    decision_center
)

#添加依赖
ament_target_dependencies(${target}
    ${DEPENDS}
)

target_link_libraries(${target}
    config_reader //配置文件读取器
    vehicle_info //车辆信息
)

#安装动态库、节点和相关目录
install(TARGETS
    ${target}
    LIBRARY DESTINATION lib)
