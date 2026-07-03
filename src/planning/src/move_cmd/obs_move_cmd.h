#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning
{
    class ObsMoveCmd : public rclcpp::Node //障碍车运动指令
    {
    public:
        ObsMoveCmd();
    private:

    };
}  // namespace Planning
#endif  // OBS_MOVE_CMD_H_
