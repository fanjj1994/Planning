#include "planning_process.h"

namespace Planning {
  PlanningProcess::PlanningProcess() : Node("planning_process")
  {
    // Initialize the planning process
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
    rclcpp::Rate rate(1.0);
    rate.sleep();

    if (initPlanning() == false)
    {
      RCLCPP_ERROR(this->get_logger(), "planning init failed!");
      return false;
    }

    // planning main process
    else
    {
      RCLCPP_INFO(this->get_logger(), "init complete. Start planning process main function!");
    }
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
    // classify client type
    std_string serverName;
    if constexpr (std::is_same_v<T, rclcpp::Client<base_msgs::srv::PNCMapService>::SharedPtr> == true)
    {
      serverName = "pnc_map";
    }
    else if constexpr (std::is_same_v<T, rclcpp::Client<base_msgs::srv::GlobalPathService>::SharedPtr> == true)
    {
      serverName = "global_path";
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "wrong client type!");
    }

    // wait_for_server
    while (!client->wait_for_service(1s)) // wait for 1 second
    {
      if (!rclcpp::ok()) // Ctrl+C to stop
      {
        RCLCPP_ERROR(this->get_logger(), "Interruped while waiting for the %s server.", serverName.c_str());
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Server %s not available, waiting again...", serverName.c_str());
    }
    return true;
  }

  boolean PlanningProcess::requestPNCMap()
  {
    RCLCPP_INFO(this->get_logger(), "Send request to PNCMapServer");

    // send request
    auto request = std::make_shared<base_msgs::srv::PNCMapService::Request>();
    request->map_type = configReaderProcess->getPNCMap().type_;

    // receive response
    auto result = pncMapClient->async_send_request(request);

    // identify response status
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Map server responds successfully!");
      pncMap = result.get()->pnc_map; // get the map from the response
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive response from PNCMapServer");
      return false;
    }
    return false;
  }

  boolean PlanningProcess::requestGlobalPath()
  {
    RCLCPP_INFO(this->get_logger(), "Send request to GlobalPathServer");

    // send request
    auto request = std::make_shared<base_msgs::srv::GlobalPathService::Request>();
    request->pnc_map = pncMap;
    request->global_planner_type = configReaderProcess->getGlobalPath().type_;

    if (request->pnc_map.midline.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "PNCMap is empty before sending to GlobalPathServer");
    }

    // receive response
    auto result = globalPathClient->async_send_request(request);

    // identify response status
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Global path server responds successfully!");
      globalPath = result.get()->global_path; // get the global path from the response
      return true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to receive response from GlobalPathServer");
      return false;
    }
    return false;
  }

} // namespace Planning