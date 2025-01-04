#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "curve.h"

namespace Planning {
  class VehicleInfoBase : public rclcpp::Node
  {
  public:
    VehicleInfoBase();

  private:
  };
} // namespace Planning
#endif // ! VEHICLE_INFO_BASE_H_
