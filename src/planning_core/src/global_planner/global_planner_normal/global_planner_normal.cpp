#include "global_planner_normal.h"

namespace Planning
{
  GlobalPlannerNormal::GlobalPlannerNormal()
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path"), "GlobalPlannerNormal is running");

    globalPathPlannerConfig = std::make_unique<ConfigReader>();
    globalPathPlannerConfig->readGlobalPathConfig();

    globalPlannerType = static_cast<uint8>(GlobalPlannerType::NORMAL);
  }

  nav_msgs::msg::Path GlobalPlannerNormal::searchGlobalPath(const base_msgs::msg::PNCMap &pncMap)
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path"), "Using Normal Global Planner");

    globalPath.header.frame_id = pncMap.header.frame_id;
    globalPath.header.stamp = rclcpp::Clock().now();
    globalPath.poses.clear();

    geometry_msgs::msg::PoseStamped p_tmp;
    p_tmp.header = pncMap.header;
    p_tmp.pose.orientation.x = 0.0;
    p_tmp.pose.orientation.y = 0.0;
    p_tmp.pose.orientation.z = 0.0;
    p_tmp.pose.orientation.w = 0.0;

    const uint8 midLineSize = pncMap.midline.points.size();
    for (uint8 i = 0U; i < midLineSize; i++)
    {
      p_tmp.pose.position.x = (pncMap.midline.points[i].x + pncMap.right_boundary.points[i].x) / 2.0;
      p_tmp.pose.position.y = (pncMap.midline.points[i].y + pncMap.right_boundary.points[i].y) / 2.0;
      globalPath.poses.emplace_back(p_tmp);
    }

    RCLCPP_INFO(rclcpp::get_logger("global_path"), "Global path created, points size: %ld", globalPath.poses.size());

    return globalPath;
  }
} // namespace Planning