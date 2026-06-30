cmake_minimum_required(VERSION 3.8)
project(planning)

#设置依赖
set(DEPENDS
    "rclcpp"
    "base_msgs"
    "geometry_msgs"
    "nav_msgs"
    "visualization_msgs"// 数据接口：提供一组专门为了可视化的消息类型的数据
)

#添加动态库
add_library(global_planner SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/global_planner_normal/global_planner_nomal.cpp
)

#添加节点
add_executable(global_path_server
    ${CMAKE_CURRENT_SOURCE_DIR}/global_path_server.cpp
)

set(project_targets
    global_planner
    global_path_server
)

#添加依赖
foreach(target ${project_targets})
    ament_target_dependencies(${target}
        ${DEPENDS}
    )
endforeach()

target_link_libraries(global_planner
    config_reader
)

target_link_libraries(global_path_server
    global_planner
)

#安装动态库、节点和相关目录
install(TARGETS
    global_planner
    LIBRARY DESTINATION lib)

install(TARGETS
    global_path_server
    DESTINATION lib/${PLANNING_PACKAGE_NAME})
    