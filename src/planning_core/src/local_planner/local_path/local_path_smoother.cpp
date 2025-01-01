#include "local_path_smoother.h"

namespace Planning
{
    LocalPathSmoother::LocalPathSmoother()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_path"), "LocalPathSmoother initialized");
    }
}  // namespace Planning