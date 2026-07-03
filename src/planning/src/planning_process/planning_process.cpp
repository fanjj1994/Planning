#include "planning_process.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
  {
        RCLCPP_INFO(this->get_logger(), "PlanningProcess node has been created.");
  }

  bool PlanningProcess::process() //总流程
  {
    return true;
  }

}  // namespace Planning
