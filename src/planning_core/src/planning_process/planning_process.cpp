#include "planning_process.h"

namespace Planning {
  PlanningProcess::PlanningProcess() : Node("planning_process")
  {
    // Initialize the planning process
    RCLCPP_INFO(this->get_logger(), "PlanningProcess initialized!");
    RCLCPP_INFO(this->get_logger(), "PlanningProcess created!");

    // Read config file for planning process
    configReaderProcess = std::make_unique<ConfigReader>();
    configReaderProcess->readProcessConfig();
    auto obsDis = configReaderProcess->getProcess().obs_dis_;
  }

  boolean PlanningProcess::process()
  {
    return true;
  }

} // namespace Planning