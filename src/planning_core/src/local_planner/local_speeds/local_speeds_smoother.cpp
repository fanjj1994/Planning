#include "local_speeds_smoother.h"

namespace Planning
{
  LocalSpeedsSmoother::LocalSpeedsSmoother()
  {
    RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "LocalSpeedsSmoother initialized");

    // Read config file for local speeds smoother component
    localSpeedsSmootherConfigReader = std::make_unique<ConfigReader>();
    localSpeedsSmootherConfigReader->readLocalSpeedsConfig();
  }
  void LocalSpeedsSmoother::smoothLocalSpeeds(base_msgs::msg::LocalSpeeds &localSpeeds)
  {
    // Currently decide not to smooth local speeds. This function is reserved for future use when we want to implement
    // local speeds smoothing logic.
    RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "Local speeds smoothed");
    (void)localSpeeds;
  }
} // namespace Planning
