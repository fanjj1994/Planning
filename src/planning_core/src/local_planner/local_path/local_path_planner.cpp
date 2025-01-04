#include "local_path_planner.h"

namespace Planning {
  LocalPathPlanner::LocalPathPlanner()
  {
    RCLCPP_INFO(rclcpp::get_logger("local_path"), "LocalPathPlanner initialized");
  }
} // namespace Planning