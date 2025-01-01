cmake_minimum_required(VERSION 3.22)
project(common)

# set config_reader dependency
set(CONFIG_DEPENDS
  "rclcpp"
  "yaml-cpp"
  "ament_index_cpp"
)

# set Maths dependency
set(MATH_DEPENDS
  "rclcpp"
  "base_msgs"
  "geometry_msgs"
  "nav_msgs"
)

# add dynamic library
add_library(config_reader SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/config_reader/config_reader.cpp
)

add_library(mathlibs SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/math/curve.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/math/polynomial_curve.cpp
)

set(config_target
  config_reader)

set(math_target
  mathlibs)

# add dependencies
ament_target_dependencies(${config_target} ${CONFIG_DEPENDS})
ament_target_dependencies(${math_target} ${MATH_DEPENDS})

target_link_libraries(${config_target} yaml-cpp)
target_link_libraries(${math_target} Eigen3::Eigen)

# install dynamic lib, node and relevant directory
install(TARGETS ${config_target} ${math_target}
  LIBRARY DESTINATION lib
)