#include "ego_car_move_cmd.h"

namespace Planning {
    EgoCarMoveCmd::EgoCarMoveCmd() : Node("ego_car_move_cmd_node")
    {
        RCLCPP_INFO(this->get_logger(), "EgoCarMoveCmd Node has been created.");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::EgoCarMoveCmd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}