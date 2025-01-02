#include "global_planner_normal.h"

namespace Planning
{
    GlobalPlannerNormal::GlobalPlannerNormal()
    {
        RCLCPP_INFO(rclcpp::get_logger("global_path "), "GlobalPlannerNormal is running");
    }
}  // namespace Planning