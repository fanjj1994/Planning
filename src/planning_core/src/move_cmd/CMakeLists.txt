cmake_minimum_required(VERSION 3.22)
project(move_cmd)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "base_msgs"
)


# add node
add_executable(ego_car_move_cmd
 ${CMAKE_CURRENT_SOURCE_DIR}/ego_car_move_cmd.cpp
)

add_executable(tp_move_cmd
 ${CMAKE_CURRENT_SOURCE_DIR}/tp_move_cmd.cpp
)

set(MOVE_TARGETS
  ego_car_move_cmd
  tp_move_cmd
)

# add dependencies
foreach(target ${MOVE_TARGETS})
  ament_target_dependencies(${target} ${DEPENDS})
  # TODO: add more dependencies
  # target_link_libraries(${target}
  #   config_reader
  #   vehicle_info
  # )
endforeach()


# install dynamic lib, node and relevant directory
install(TARGETS ${MOVE_TARGETS}
  DESTINATION lib/${PLANNING_PACKAGE_NAME}
)