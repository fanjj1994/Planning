#include "planning_process.h"

namespace Planning {
  PlanningProcess::PlanningProcess() : Node("planning_node")
  {
    // Initialize the planning process
    RCLCPP_INFO(this->get_logger(), "PlanningProcess initialized!");
    RCLCPP_INFO(this->get_logger(), "Planning_node created!");
  }

  boolean PlanningProcess::process()
  {
    return true;
  }

} // namespace Planning