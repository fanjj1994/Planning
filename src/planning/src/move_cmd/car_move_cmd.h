#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class CarMoveCmd : public rclcpp::Node //主车运动指令
    {
    public:
        CarMoveCmd();
    private:

    };
}  // namespace Planning
#endif  // CAR_MOVE_CMD_H_
