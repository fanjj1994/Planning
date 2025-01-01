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

# TODO: add dynamic library


# TODO: install dynamic lib, node and relevant directory


