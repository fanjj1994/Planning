#ifndef LOCAL_SPEEDS_SMOOTHER_H_
#define LOCAL_SPEEDS_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "config_reader.h"

namespace Planning
{
  class LocalSpeedsSmoother
  {
  public:
    LocalSpeedsSmoother();
    LocalSpeedsSmoother(const LocalSpeedsSmoother &) = delete;
    LocalSpeedsSmoother operator=(const LocalSpeedsSmoother &) = delete;
    ~LocalSpeedsSmoother() = default;

    void smoothLocalSpeeds(base_msgs::msg::LocalSpeeds &localSpeeds);

  private:
    std::unique_ptr<ConfigReader> localSpeedsSmootherConfigReader; // config reader for local speeds smoother
  };
} // namespace Planning
#endif // ! LOCAL_SPEEDS_SMOOTHER_H_
