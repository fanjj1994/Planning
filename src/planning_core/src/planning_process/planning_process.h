#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "common_type.h"
#include "config_reader.h"
#include "ego_car_base.h"
#include "tp_base.h"
#include "reference_line_creator.h"
#include "decision_center.h"
#include "local_path_planner.h"
#include "local_speeds_planner.h"
#include "local_trajectory_combiner.h"

#include <vector>
#include <cmath>
#include <algorithm>

namespace Planning {
  static constexpr uint8 PROCESS_FAILURE{ 255U };

  class PlanningProcess : public rclcpp::Node
  {
  public:
    PlanningProcess();
    ~PlanningProcess() = default;
    boolean process();

  private:
  };
} // namespace Planning
#endif // !PLANNING_PROCESS_H_