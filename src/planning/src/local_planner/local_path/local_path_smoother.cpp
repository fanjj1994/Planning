#include "local_path_smoother.h"

namespace Planning
{
  LocalPathSmoother::LocalPathSmoother()//局部路径平滑器
  {
    RCLCPP_INFO(rclcpp::get_logger("local_path_smoother"), "local_path_smoother created");
  }
}// namespace Planning
