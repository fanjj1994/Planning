#include "global_path_server.h"

namespace Planning {
  GlobalPathServer::GlobalPathServer() : Node("global_path_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "GlobalPathServer is running");

    // global path publisher
    globalPathPublisher = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    globalPathRvizPublisher = this->create_publisher<visualization_msgs::msg::Marker>("global_path_rviz", 10);

    // global path rviz publisher
    globalPathServer = this->create_service<base_msgs::srv::GlobalPathService>(
        "global_path_server",
        std::bind(&GlobalPathServer::responseGlobalPathCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void GlobalPathServer::responseGlobalPathCallback(
      const std::shared_ptr<base_msgs::srv::GlobalPathService::Request> request,
      const std::shared_ptr<base_msgs::srv::GlobalPathService::Response> response) // response global path callback
  {
    // Step1: receive request
    switch (request->global_planner_type)
    {
    case static_cast<uint8>(GlobalPlannerType::NORMAL):
      globalPlannerCreator = std::make_shared<GlobalPlannerNormal>();
      break;
    case static_cast<uint8>(GlobalPlannerType::ASTAR):
      // TODO: Design A* global planner
      // globalPlannerCreator = std::make_shared<GlobalPlannerAStar>();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "GlobalPathServer: Undefined global planner type!");
      return;
    }
    if (request->pnc_map.midline.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "pnc_map is empty! Global path cannot be created due to dependency!");
      return;
    }
    // Step2: search & respond global path
    const auto globalPath = globalPlannerCreator->searchGlobalPath(request->pnc_map);
    response->global_path = globalPath;

    // Step3: publish global path (for local planning usage)
    globalPathPublisher->publish(globalPath);
    RCLCPP_INFO(this->get_logger(), "global path published");

    // Step4: publish global path (for rviz display)
    const auto globalPathRviz = path2Marker(globalPath);
    globalPathRvizPublisher->publish(globalPathRviz);
    RCLCPP_INFO(this->get_logger(), "global path for rviz published");
  }

  visualization_msgs::msg::Marker GlobalPathServer::path2Marker(const nav_msgs::msg::Path& path) // path to marker
  {
    visualization_msgs::msg::Marker pathRviz;
    // TODO: path2Marker implementation
    return pathRviz;
  }

} // namespace Planning

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::GlobalPathServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
