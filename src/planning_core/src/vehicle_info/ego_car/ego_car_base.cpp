#include "ego_car_base.h"

namespace Planning {
  EgoCar::EgoCar()
  {
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "EgoCar is initialized");
  }

} // namespace Planning