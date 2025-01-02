#include "decision_center.h"

namespace Planning {
  DecisionCenter::DecisionCenter()
  {
    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "DecisionCenter is running");
  }
} // namespace Planning