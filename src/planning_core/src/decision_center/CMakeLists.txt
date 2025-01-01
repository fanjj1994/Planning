cmake_minimum_required(VERSION 3.22)
project(decision_center)

# set config_reader dependency
set(DEPENDS
  "rclcpp"
)



# add dynamic library
add_library(decision_center SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/decision_center.cpp
)

set(target
  decision_center
)

# TODO: add dependencies
ament_target_dependencies(${target} ${DEPENDS})

# TODO: add link libraries
# target_link_libraries(${target} 
#   config_reader
#   vehicle_info
# )

# install dynamic lib, node and relevant directory
install(TARGETS ${target}
  LIBRARY DESTINATION lib 
)


