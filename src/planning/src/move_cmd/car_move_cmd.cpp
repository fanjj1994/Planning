#include "car_move_cmd.h"

namespace Planning
{
  CarMoveCmd::CarMoveCmd() : Node("car_move_cmd_node") //主车运动指令
  {
    RCLCPP_INFO(this->get_logger(), "car_move_cmd_node created");
  }
} // namespace Planning