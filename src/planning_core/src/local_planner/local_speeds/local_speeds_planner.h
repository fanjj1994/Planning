#ifndef LOCAL_SPEEDS_PLANNER_H_
#define LOCAL_SPEEDS_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_speeds_point.hpp"
#include "base_msgs/msg/local_speeds.hpp"
#include "config_reader.h"
#include "polynomial_curve.h"
#include "decision_center.h"
#include "local_speeds_smoother.h"

namespace Planning
{
  class LocalSpeedsPlanner
  {
  public:
    LocalSpeedsPlanner();
    LocalSpeedsPlanner(const LocalSpeedsPlanner &) = delete;
    LocalSpeedsPlanner operator=(const LocalSpeedsPlanner &) = delete;
    ~LocalSpeedsPlanner() = default;

    base_msgs::msg::LocalSpeeds planLocalSpeeds(const std::shared_ptr<DecisionCenter> &decision);

    void initializeLocalSpeeds();

  private:
    std::unique_ptr<ConfigReader> localSpeedsPlannerConfigReader; // configuration reader for local speeds planner
    base_msgs::msg::LocalSpeeds localSpeeds;                      // Cached local speeds to be published
    std::shared_ptr<LocalSpeedsSmoother> localSpeedsSmoother;     // local speeds smoother
  };
} // namespace Planning
#endif // ! LOCAL_SPEEDS_PLANNER_H_
