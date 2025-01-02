#include "pnc_map_server.h"

namespace Planning {
  PNCMapServer::PNCMapServer() : Node("pnc_map_server_node")
  {
    RCLCPP_INFO(this->get_logger(), "PNCMapServer is running");
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
