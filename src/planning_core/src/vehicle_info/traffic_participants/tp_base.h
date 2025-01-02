#ifndef TP_BASE_H_
#define TP_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "vehicle_info_base.h"

namespace Planning
{
    class TP: public VehicleInfoBase
    {
    public:
        TP();
        TP(const TP&) = delete;
        TP& operator=(const TP&) = delete;
        ~TP() = default;

    private:

    };
}  // namespace Planning
#endif  // ! TP_BASE_H_
