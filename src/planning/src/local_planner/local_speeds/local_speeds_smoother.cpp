#include "local_speeds_smoother.h"

namespace Planning
{
  LocalSpeedsSmoother::LocalSpeedsSmoother()//速度平滑规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("local_speeds_smoother"), "local_speeds_smoother created");
  }
}// namespace Planning