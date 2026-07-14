#include "global_planner_normal.h"

namespace Planning
{
  GlobalPlannerNormal::GlobalPlannerNormal() // 普通全局路径规划器
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path"), "global_planner_normal created");

    //读取配置文件，并给参数赋值
    global_planner_config_ = std::make_unique<ConfigReader>();
    global_planner_config_->read_global_path_config();
    global_planner_type_ = static_cast<int>(GlobalPlannerType::NORMAL);

  }

  Path GlobalPlannerNormal::search_global_path(const PNCMap &pnc_map)//搜索全局路径
  {
    RCLCPP_INFO(rclcpp::get_logger("global_path"),"using normal global_planner");

    global_path_.header.frame_id = pnc_map.header.frame_id;
    global_path_.header.stamp = rclcpp::Clock().now();
    global_path_.poses.clear();

    PoseStamped p_tmp;
    p_tmp.header = global_path_.header;
    p_tmp.pose.orientation.x = 0.0;
    p_tmp.pose.orientation.y = 0.0;
    p_tmp.pose.orientation.z = 0.0;
    p_tmp.pose.orientation.w = 1.0;

    const int midline_size = pnc_map.midline.points.size();
    for (int i = 0;i < midline_size; i++)
    {
      p_tmp.pose.position.x = (pnc_map.midline.points[i].x + pnc_map.right_boundary.points[i].x)/2.0;
      p_tmp.pose.position.y = (pnc_map.midline.points[i].y + pnc_map.right_boundary.points[i].y)/2.0;
      global_path_.poses.emplace_back(p_tmp);
    }

    RCLCPP_INFO(rclcpp::get_logger("global_path"),"global_path created, points size: %ld",global_path_.poses.size());
    return global_path_;
  }

}// namespace Planning
