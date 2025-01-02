#ifndef LOCAL_SPEEDS_SMOOTHER_H_
#define LOCAL_SPEEDS_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class LocalSpeedsSmoother
    {
    public:
        LocalSpeedsSmoother();
        LocalSpeedsSmoother(const LocalSpeedsSmoother&) = delete;
        LocalSpeedsSmoother operator=(const LocalSpeedsSmoother&) = delete;
        ~LocalSpeedsSmoother() = default;
    private:

    };
}  // namespace Planning
#endif  // ! LOCAL_SPEEDS_SMOOTHER_H_
