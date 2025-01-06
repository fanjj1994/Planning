#include "planning_process.h"

namespace Planning {
  PlanningProcess::PlanningProcess() : Node("planning_node")
  {
    // Initialize the planning process
    RCLCPP_INFO(this->get_logger(), "PlanningProcess initialized!");
    RCLCPP_INFO(this->get_logger(), "Planning_node created!");

    // Read config file for planning process
    configReaderProcess = std::make_unique<ConfigReader>();
    configReaderProcess->readProcessConfig();
    auto obsDis = configReaderProcess->getProcess().obs_dis_;
    RCLCPP_INFO(this->get_logger(), "obs_dis: %f", obsDis);
  }

  boolean PlanningProcess::process()
  {
    return true;
  }

} // namespace Planning