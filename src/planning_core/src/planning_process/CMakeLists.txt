cmake_minimum_required(VERSION 3.22)
project(planning_process)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "nav_msgs"
  "base_msgs"
  "tf2"
  "tf2_ros"
)

# add node
message("current planning_node source dir: " ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(planning_node
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/planning_process.cpp
)

set(target 
  planning_node
)

# add dependencies
ament_target_dependencies(${target} 
  ${DEPENDS}
)

# target_link_libraries(${target}
#   config_reader
# )

# install dynamic lib, node and relevant directory
install(TARGETS 
  ${target}
  DESTINATION lib/${PLANNING_PACKAGE_NAME})