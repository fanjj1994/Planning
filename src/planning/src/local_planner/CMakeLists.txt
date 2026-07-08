cmake_minimum_required(VERSION 3.28)
project(local_planner)

# 设置依赖
set(DEPENDS 
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
  "tf2"
)

# 添加动态库
add_library(local_planner SHARED 
  ${CMAKE_CURRENT_SOURCE_DIR}/local_trajectory_combiner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_path/local_path_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_path/local_path_smoother.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_speeds/local_speeds_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_speeds/local_speeds_smoother.cpp
)

set(target 
  local_planner
)

# 添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target} 
  config_reader
  mathlibs
  vehicle_info
  decision_center
)

# 安装动态库、节点和相关目录
install(TARGETS 
  ${target}
  LIBRARY DESTINATION lib)
