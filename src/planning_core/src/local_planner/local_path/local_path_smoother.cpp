#include "local_path_smoother.h"

namespace Planning
{
  LocalPathSmoother::LocalPathSmoother()
  {
    RCLCPP_INFO(rclcpp::get_logger("local_path"), "LocalPathSmoother initialized");

    // Read config file for local path smoother component
    localPathConfigReader = std::make_unique<ConfigReader>();
    localPathConfigReader->readLocalPathConfig();
  }

  void LocalPathSmoother::smoothLocalPath(base_msgs::msg::LocalPath &localPath)
  {
    // Currently decide not to smooth local path. This function is reserved for future use when we want to implement
    // local path smoothing logic.
    RCLCPP_INFO(rclcpp::get_logger("local_path"), "Local path smoothed");
    (void)localPath;
  }
} // namespace Planning
