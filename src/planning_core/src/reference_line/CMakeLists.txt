cmake_minimum_required(VERSION 3.22)
project(reference_line)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "base_msgs"
)

# add dynamic lib
add_library(reference_line SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_creator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/reference_line_smoother.cpp
)

set(target
  reference_line
)

# add dependencies
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  mathlibs
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)

# install dynamic lib, node and relevant directory
install(TARGETS 
  ${target}
  LIBRARY DESTINATION lib
)
