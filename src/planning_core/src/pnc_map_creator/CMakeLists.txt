cmake_minimum_required(VERSION 3.22)
project(pnc_map_creator)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "geometry_msgs"
  "visualization_msgs"
  "base_msgs"
)

# add dynamic lib
add_library(pnc_map_creator SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_straight/pnc_map_creator_straight.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_sturn/pnc_map_creator_sturn.cpp
)

set(LIB_TARGETS
  pnc_map_creator
)

# add node
add_executable(pnc_map_server
  ${CMAKE_CURRENT_SOURCE_DIR}/pnc_map_server.cpp
)

set(NODE_TARGETS 
  pnc_map_server
)

set(PROJECT_TARGETS
  pnc_map_creator
  pnc_map_server
)
# add dependencies
foreach(target ${PROJECT_TARGETS})
  ament_target_dependencies(${target}
    ${DEPENDS}
  )
endforeach()

target_link_libraries(pnc_map_server
  config_reader
  pnc_map_creator
)

# install dynamic lib, node and relevant directory
install(TARGETS 
  pnc_map_server
  DESTINATION lib/${PLANNING_PACKAGE_NAME})
