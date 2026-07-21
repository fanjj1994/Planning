cmake_minimum_required(VERSION 3.28)
project(test)

#设置依赖
set(DEPENDS
    "rclcpp"
)

#添加节点
add_executable(osqp_test
    ${CMAKE_CURRENT_SOURCE_DIR}/osqp_test/osqp_test.cpp
)

#添加依赖
ament_target_dependencies(osqp_test
    ${DEPENDS}
)

target_link_libraries(osqp_test
    Eigen3::Eigen
    OsqpEigen::OsqpEigen
)

#安装动态库、节点和相关目录
install(TARGETS
    osqp_test
    DESTINATION lib/${PLANNING_PACKAGE_NAME})
