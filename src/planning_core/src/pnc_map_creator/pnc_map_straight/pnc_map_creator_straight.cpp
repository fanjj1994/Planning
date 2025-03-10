#include "pnc_map_creator_straight.h"

namespace Planning {
  PNCMapCreatorStraight::PNCMapCreatorStraight()
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PNCMapCreatorStraight is running");
  }

  base_msgs::msg::PNCMap PNCMapCreatorStraight::createPNCMap()
  {
    return base_msgs::msg::PNCMap();
  }

} // namespace Planning