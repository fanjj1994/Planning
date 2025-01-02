cmake_minimum_required(VERSION 3.22)
project(vehicle_info)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "base_msgs"
)

# add dynamic lib
add_library(vehicle_info SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/ego_car/ego_car_base.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/traffic_participants/tp_base.cpp
)

set(target
  vehicle_info
)

# add dependencies
ament_target_dependencies(${target}
  ${DEPENDS}
)

target_link_libraries(${target}
  config_reader
  mathlibs
)

# install dynamic lib, node and relevant directory
install(TARGETS ${target}
  LIBRARY DESTINATION lib
)


