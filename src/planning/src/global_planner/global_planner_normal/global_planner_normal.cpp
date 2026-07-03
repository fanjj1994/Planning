#include "global_planner_normal.h"

namespace Planning
{
  GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局路径规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_planner_normal created");
  }

}// namespace Planning