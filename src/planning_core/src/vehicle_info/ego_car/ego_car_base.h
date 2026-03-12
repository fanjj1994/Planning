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
    EgoCar(const EgoCar &) = delete;
    EgoCar &operator=(const EgoCar &) = delete;
    ~EgoCar() = default;

    void vehicleCartesianToFrenet(const base_msgs::msg::Referline &referenceline) override;
    void vehicleCartesianToFrenet2Path(const base_msgs::msg::LocalPath &localPath,
                                       const base_msgs::msg::Referline &referenceline,
                                       const std::shared_ptr<VehicleInfoBase> &egoCar) override;

  private:
  };
} // namespace Planning
#endif // ! EGO_CAR_BASE_H_
