#include "tp_move_cmd.h"

namespace Planning {
  TPMoveCmd::TPMoveCmd() : Node("tp_move_cmd_node")
  {
    RCLCPP_INFO(this->get_logger(), "TPMoveCmd Node has been created.");
  }
} // namespace Planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Planning::TPMoveCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}