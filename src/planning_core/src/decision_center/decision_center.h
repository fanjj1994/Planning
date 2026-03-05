#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "ego_car_base.h"
#include "tp_base.h"
#include <vector>

namespace Planning
{
  enum class SLPointType : uint8 // decision point type
  {
    DECISION_LEFT_PASS = 0U,   // deicide to pass from left side
    DECISION_RIGHT_PASS = 1U,  // decide to pass from right side
    DEISION_STOP = 2U,         // decide to stop the car at this point
    DEISION_START = 3U,        // start point of the decision
    DECISION_END = 4U          // end point of the decision
  };
  class DecisionCenter
  {
  public:
    DecisionCenter();
    ~DecisionCenter() = default;

  private:
  };
} // namespace Planning
#endif // ! DECISION_CENTER_H_
