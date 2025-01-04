cmake_minimum_required(VERSION 3.22)
project(local_planner)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "base_msgs"
  "tf2"
)

# add dynamic library
add_library(${PROJECT_NAME} SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/local_trajectory_combiner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_path/local_path_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_path/local_path_smoother.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_speeds/local_speeds_planner.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/local_speeds/local_speeds_smoother.cpp
)

set(target
  local_planner
)

# add dependencies
ament_target_dependencies(${target}
  ${DEPENDS}
)

# add dependencies
target_link_libraries(${target}
  config_reader
  mathlibs
  vehicle_info
  decision_center
)

# install dynamic lib, node and relevant directory
install(TARGETS 
  ${target}
  LIBRARY DESTINATION lib
)


