#include "global_path_server.h"

namespace Planning
{
    GlobalPathServer::GlobalPathServer() : Node("global_path_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "GlobalPathServer is running");
    }

}  // namespace Planning

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
