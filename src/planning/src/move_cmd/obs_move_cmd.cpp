#include "obs_move_cmd.h"

namespace Planning
{
  ObsMoveCmd::ObsMoveCmd() : Node("obs_move_cmd_node") //障碍车运动指令
  {
    RCLCPP_INFO(this->get_logger(), "obs_move_cmd_node created");
  }
}// namespace Planning