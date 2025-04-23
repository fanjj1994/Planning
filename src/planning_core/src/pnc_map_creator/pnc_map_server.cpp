#include "pnc_map_server.h"

namespace Planning {
  PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "PNCMapServer is running");

    // map publisher
    mapPublisher = this->create_publisher<base_msgs::msg::PNCMap>("pnc_map", 10U);
    mapRvizPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("pnc_map_marker_array", 10U);

    // map server
    mapServer = this->create_service<base_msgs::srv::PNCMapService>(
        "pnc_map_server",
        std::bind(&PNCMapServer::responsePNCMapCallBack, this, std::placeholders::_1, std::placeholders::_2));
  }

  // response & publish map
  void PNCMapServer::responsePNCMapCallBack(const std::shared_ptr<base_msgs::srv::PNCMapService::Request> request,
                                            const std::shared_ptr<base_msgs::srv::PNCMapService::Response> response)
  {
    // Step1: receive request
    switch (request->map_type)
    {
    case static_cast<uint8>(PNCMapType::STRAIGHT):
      mapCreator = std::make_shared<PNCMapCreatorStraight>();
      RCLCPP_INFO(this->get_logger(), "Straight Map Creator is selected");
      break;
    case static_cast<uint8>(PNCMapType::STURN):
      mapCreator = std::make_shared<PNCMapCreatorSTurn>();
      RCLCPP_INFO(this->get_logger(), "STurn Map Creator is selected");
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "PNCMapServer: Undefined map type!");
      return;
    }

    // Step2: respond & create map
    const auto pncMap = mapCreator->createPNCMap();
    response->pnc_map = pncMap;

    if (pncMap.midline.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "PNCMapCreator failed to generate midline points!");
    }

    // Step3: publish map (planning node)
    mapPublisher->publish(pncMap);
    RCLCPP_INFO(this->get_logger(), "PNCMapServer: publish map");

    // Step4: publish map (rviz)
    const auto pncMapMarkerArray = mapCreator->getPNCMapMarkerArray();
    mapRvizPublisher->publish(pncMapMarkerArray); // publish map for rviz
    RCLCPP_INFO(this->get_logger(), "PNCMapServer: publish map for rviz");
  }

} // namespace Planning

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::PNCMapServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
