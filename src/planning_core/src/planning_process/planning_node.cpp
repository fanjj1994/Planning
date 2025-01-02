#include "planning_process.h"
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("planning_node"), "planning start");
  auto node = std::make_shared<Planning::PlanningProcess>();
  if (node->process() == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger("planning_node"), "planning failed!");
    rclcpp::shutdown();
    return Planning::PROCESS_FAILURE;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
