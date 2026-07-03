cmake_minimum_required(VERSION 3.28)
project(move_cmd)

# 设置依赖
set(DEPENDS 
  "rclcpp"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "base_msgs"
)

# # 添加节点
# add_executable(car_move_cmd 
#   ${CMAKE_CURRENT_SOURCE_DIR}/car_move_cmd.cpp
# )

# add_executable(obs_move_cmd 
#   ${CMAKE_CURRENT_SOURCE_DIR}/obs_move_cmd.cpp
# )

# set(MOVE_TARGETS 
#   car_move_cmd
#   obs_move_cmd
# )

# # 添加依赖
# foreach(target ${MOVE_TARGETS})
#   ament_target_dependencies(${target}
#     ${DEPENDS}
#   )

#   target_link_libraries(${target} 
#     config_reader  
#     vehicle_info
#   )
# endforeach()

# # 安装动态库、节点和相关目录
# install(TARGETS 
#   ${MOVE_TARGETS}
#   DESTINATION lib/${PLANNING_PACKAGE_NAME})
