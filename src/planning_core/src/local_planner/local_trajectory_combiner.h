#ifndef LOCAL_TRAJECTORY_COMBINER_H_
#define LOCAL_TRAJECTORY_COMBINER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "base_msgs/msg/local_trajectory_point.hpp"
#include "config_reader.h"

namespace Planning
{
  class LocalTrajectoryCombiner
  {
  public:
    LocalTrajectoryCombiner();
    LocalTrajectoryCombiner(const LocalTrajectoryCombiner &) = delete;
    LocalTrajectoryCombiner operator=(const LocalTrajectoryCombiner &) = delete;
    ~LocalTrajectoryCombiner() = default;

    inline base_msgs::msg::LocalTrajectory getLocalTrajectory() const { return localTrajectory; }
    base_msgs::msg::LocalTrajectory combineLocalTrajectory(const base_msgs::msg::LocalPath &localPath,
                                                           const base_msgs::msg::LocalSpeeds &localSpeeds);

  private:
    std::unique_ptr<ConfigReader> trajectoryCombinerConfigReader; // configuration reader for local
                                                                  // trajectory combiner
    base_msgs::msg::LocalTrajectory localTrajectory; // local trajectory consisting of path and speed information
  };
} // namespace Planning
#endif // ! LOCAL_TRAJECTORY_COMBINER_H_
