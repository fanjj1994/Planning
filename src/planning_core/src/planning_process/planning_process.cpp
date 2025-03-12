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
    obsDis = configReaderProcess->getProcess().obs_dis_;

    // create PNCMap & Global Path Client
    pncMapClient = this->create_client<base_msgs::srv::PNCMapService>("pnc_map_server");
    globalPathClient = this->create_client<base_msgs::srv::GlobalPathService>("global_path_server");
  }

  boolean PlanningProcess::process()
  {
    // set 1 second timer to wait rviz2 & xacro to be ready
    rclcpp::Rate rate(1s);
    rate.sleep();

    if (initPlanning() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "planning init failed!");
      return false;
    }
    
    // planning main process

    return true;
  }

  boolean PlanningProcess::initPlanning()
  {
    boolean initPlanningResult = true;
    // TODO: create Tp

    // connect map server
    if (connectServer(pncMapClient) == false)
    {
      RCLCPP_ERROR(this->get_logger(), "connect pnc map server failed!");
      initPlanningResult = false;
    } 

    // get map
    if (requestPNCMap() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "request pnc map failed!");
      initPlanningResult = false;
    }

    // connect global path server
    if (connectServer(globalPathClient) == false)
    {
      RCLCPP_ERROR(this->get_logger(), "connect global path server failed!");
      initPlanningResult = false;
    }

    // get global path
    if (requestGlobalPath() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "request global path failed!");
      initPlanningResult = false;
    }
    
    return initPlanningResult;
  }

  template <typename T>
  boolean PlanningProcess::connectServer(const T &client)
  {
    return true;
  }

  boolean PlanningProcess::requestPNCMap()
  {
    return true;
  }

  boolean PlanningProcess::requestGlobalPath()
  {
    return true;
  }

} // namespace Planning