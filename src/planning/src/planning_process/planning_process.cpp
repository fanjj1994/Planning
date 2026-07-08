#include "planning_process.h"

namespace Planning
{
  PlanningProcess::PlanningProcess() : Node("planning_process") // 规划总流程
  {
        RCLCPP_INFO(this->get_logger(), "planning_process created.");
        
        //读取配置文件
        process_config_ = std::make_unique<ConfigReader>();
        process_config_->read_planning_process_config();
        auto obs_dis = process_config_->process().obs_dis_;
  }

  bool PlanningProcess::process() //总流程
  {
    return true;
  }

}  // namespace Planning
