#ifndef PLANNING_PROCESS_H_
#define PLANNING_PROCESS_H_

#include "rclcpp/rclcpp.hpp"
#include "../common/common_type/common_type.h"

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