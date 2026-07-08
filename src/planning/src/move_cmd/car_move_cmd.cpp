#include "car_move_cmd.h"

namespace Planning
{
  CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node") //主车运动指令
  {
    RCLCPP_INFO(this->get_logger(), "car_move_cmd_node created");
  }
} // namespace Planning

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::CarMoveCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
