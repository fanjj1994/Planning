#include "local_trajectory_combiner.h"

namespace Planning
{
    LocalTrajectoryCombiner::LocalTrajectoryCombiner()
    {
        RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "LocalTrajectoryCombiner initialized");
    }
}  // namespace Planning