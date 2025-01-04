cmake_minimum_required(VERSION 3.22)
project(global_planner)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "base_msgs"
)

# add dynamic library
add_library(global_planner SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/global_planner_normal/global_planner_normal.cpp
)

# add node
add_executable(global_path_server
  ${CMAKE_CURRENT_SOURCE_DIR}/global_path_server.cpp
)

set(PROJECT_TARGETS
  global_planner
  global_path_server
)

# add dependencies
foreach(target ${PROJECT_TARGETS})
  ament_target_dependencies(${target} ${DEPENDS})
endforeach()

target_link_libraries(global_path_server
  config_reader
  global_planner
)

# install dynamic lib, node and relevant directory
install(TARGETS
  global_planner
  LIBRARY DESTINATION lib)

install(TARGETS
  global_path_server
  DESTINATION lib/${PLANNING_PACKAGE_NAME})

