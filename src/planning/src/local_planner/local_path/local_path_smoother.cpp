#include "local_path_smoother.h"

namespace Planning
{
  LocalPathSmoother::LocalPathSmoother()//局部路径平滑器
  {
    RCLCPP_INFO(rclcpp::get_logger("local_path_smoother"), "local_path_smoother created");
    
    // 读取配置文件
    local_path_config_ = std::make_unique<ConfigReader>();
    local_path_config_ -> read_local_path_config();
  }

  void LocalPathSmoother::smooth_local_path(LocalPath &path) // 平滑路径
  {
    RCLCPP_INFO(rclcpp::get_logger("local_path"),"local path smoothed");
    (void)path;
  }
} // namespace Planning
