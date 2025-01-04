#ifndef LOCAL_PATH_PLANNER_H_
#define LOCAL_PATH_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "curve.h"
#include "polynomial_curve.h"
#include "ego_car_base.h"
#include "decision_center.h"
#include "local_path_smoother.h"

namespace Planning {
  class LocalPathPlanner
  {
  public:
    LocalPathPlanner();
    LocalPathPlanner(const LocalPathPlanner&) = delete;
    LocalPathPlanner operator=(const LocalPathPlanner&) = delete;
    ~LocalPathPlanner() = default;

  private:
  };
} // namespace Planning
#endif // ! LOCAL_PATH_PLANNER_H_
