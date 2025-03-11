#include "global_planner_normal.h"

namespace Planning {
  GlobalPlannerNormal::GlobalPlannerNormal()
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path "), "GlobalPlannerNormal is running");
  }

  nav_msgs::msg::Path GlobalPlannerNormal::searchGlobalPath(const base_msgs::msg::PNCMap &)
  {
    return nav_msgs::msg::Path();
  }
} // namespace Planning