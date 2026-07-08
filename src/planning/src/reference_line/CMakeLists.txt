cmake_minimum_required(VERSION 3.28)
project(reference_line)

# 设置依赖
set(DEPENDS 
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
)

# 添加动态库
add_library(reference_line SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_creator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_smoother.cpp
)

set(target 
  reference_line
)

# 添加依赖
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target} 
  config_reader  
  mathlibs
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

# 安装动态库、节点和相关目录
install(TARGETS 
  ${target}
  LIBRARY DESTINATION lib)
