#ifndef EGO_CAR_BASE_H_
#define EGO_CAR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_base.h"

namespace Planning
{
  class EgoCar : public VehicleInfoBase
  {
  public:
    EgoCar();
    EgoCar(const EgoCar&) = delete;
    EgoCar& operator=(const EgoCar&) = delete;
    ~EgoCar() = default;

    void vehicleCartesianToFrenet(const base_msgs::msg::Referline& referenceline) override;

  private:
  };
} // namespace Planning
#endif // ! EGO_CAR_BASE_H_
