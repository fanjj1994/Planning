cmake_minimum_required(VERSION 3.8)
project(common)

#设置依赖
set(CONFIG_DEPENDS
    "rclcpp"
    "yaml-cpp"
    "ament_index_cpp"//读取配置文件
)

set(MATH_DEPENDS
    "rclcpp"//c++库
    "base_msgs"//自定义数据接口
    "geometry_msgs"//数据接口：用于表示机器人系统中的几何形状和变换
    "nav_msgs"//数据接口：用于表示机器人的导航和路径规划
)

#添加动态库
add_library(config_reader SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/config_reader/config_reader.cpp  //配置文件
)

add_library(mathlibs SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/math/curve.cpp  //曲线库
    ${CMAKE_CURRENT_SOURCE_DIR}/math/polynomial_curve.cpp // 一次多项式和五次多项式库
)

set(config_target
    config_reader
)

set(math_target
    mathlibs
)

#让动态库关联上依赖
ament_target_dependencies(${config_target}
    ${CONFIG_DEPENDS}
)

ament_target_dependencies(${math_target}
    ${MATH_DEPENDS}
)

target_link_libraries(${config_target}
    yaml-cpp
)

target_link_libraries(${math_target}
    Eigen3::Eigen
)

#安装动态库、节点和相关目录
install(TARGETS
    ${config_target}
    ${math_target}
    LIBRARY DESTINATION lib)
    