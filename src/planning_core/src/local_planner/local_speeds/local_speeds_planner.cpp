#include "local_speeds_planner.h"

namespace Planning
{
    LocalSpeedsPlanner::LocalSpeedsPlanner()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "LocalSpeedsPlanner initialized");
    }
}  // namespace Planning