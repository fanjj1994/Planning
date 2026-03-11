#include "local_trajectory_combiner.h"

namespace Planning
{
  LocalTrajectoryCombiner::LocalTrajectoryCombiner()
  {
    RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "LocalTrajectoryCombiner initialized");

    // Read config file for local trajectory combiner component
    trajectoryCombinerConfigReader = std::make_unique<ConfigReader>();
    trajectoryCombinerConfigReader->readLocalPathConfig();
    trajectoryCombinerConfigReader->readLocalSpeedsConfig();
  }

  base_msgs::msg::LocalTrajectory
  LocalTrajectoryCombiner::combineLocalTrajectory(const base_msgs::msg::LocalPath &localPath,
                                                  const base_msgs::msg::LocalSpeeds &localSpeeds)
  {
    const uint8 pathSize = static_cast<uint8>(localPath.local_path.size());
    const uint8 speedsSize = static_cast<uint8>(localSpeeds.local_speeds.size());
    localTrajectory.header = localPath.header;
    localTrajectory.local_trajectory.clear();

    // if (pathSize < 3U || speedsSize < 3U) // @todo: add speed planning and check speeds size as well
    if (pathSize < 3U)
    {
      RCLCPP_WARN(rclcpp::get_logger("local_trajectory"),
                  "Local path or speeds size is less than 3, too less points to combine a trajectory");
      return localTrajectory;
    }

    base_msgs::msg::LocalTrajectoryPoint trajectoryPt;
    for (uint8 i = 0U; i < pathSize; i++)
    {
      trajectoryPt.path_point = localPath.local_path[i]; // assign path point
      // trajectoryPt.speed_point = localSpeeds.local_speeds[i]; // @Todo: assign speed point
      localTrajectory.local_trajectory.emplace_back(trajectoryPt);
    }

    RCLCPP_INFO(rclcpp::get_logger("local_trajectory"), "Combined local trajectory with %u points", pathSize);

    return localTrajectory;
  }

} // namespace Planning
