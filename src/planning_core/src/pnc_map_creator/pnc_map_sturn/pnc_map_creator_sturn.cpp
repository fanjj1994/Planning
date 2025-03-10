#include "pnc_map_creator_sturn.h"

namespace Planning {
  PNCMapCreatorSTurn::PNCMapCreatorSTurn()
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "PNCMapCreatorSTurn is running");
  }

  base_msgs::msg::PNCMap PNCMapCreatorSTurn::createPNCMap()
  {
    return base_msgs::msg::PNCMap();
  }

} // namespace Planning