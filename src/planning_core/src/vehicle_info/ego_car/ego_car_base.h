#ifndef EGO_CAR_BASE_H_
#define EGO_CAR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_base.h"

namespace Planning
{
    class EgoCar: public VehicleInfoBase
    {
    public:
        EgoCar();
        EgoCar(const EgoCar&) = delete;
        EgoCar& operator=(const EgoCar&) = delete;
        ~EgoCar() = default;

    private:

    };
}  // namespace Planning
#endif  // ! EGO_CAR_BASE_H_
