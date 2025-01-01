cmake_minimum_required(VERSION 3.22)
project(planning_core)

# set C /C++ standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMAKE_COMPILER_IS_GNUCXX is abandoned by CMake >= 3.21
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set precompile commands, allow cmake to generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set package name
set(PLANNING_PACKAGE_NAME ${PROJECT_NAME})

include_directories(
  /usr/local/include/eigen-3.4.0
  src/common/config_reader
  src/common/math
  src/decision_center
  src/global_planner
  src/global_planner/global_planner_normal
  src/local_planner
  src/local_planner/local_path
  src/local_planner/local_speeds
  src/move_cmd
  src/planning_process
  src/pnc_map_creator
  src/pnc_map_creator/pnc_map_straight
  src/pnc_map_creator/pnc_map_sturn
  src/reference_line
  src/vehicle_info
  src/vehicle_info/ego_car
  src/vehicle_info/traffic_participants
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(base_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)

# add subdirectories
add_subdirectory(src/common)
add_subdirectory(src/decision_center)
add_subdirectory(src/global_planner)
add_subdirectory(src/local_planner)
add_subdirectory(src/move_cmd)
add_subdirectory(src/planning_process)
add_subdirectory(src/pnc_map_creator)
add_subdirectory(src/reference_line)
add_subdirectory(src/vehicle_info)

install(DIRECTORY
  config launch urdf rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
