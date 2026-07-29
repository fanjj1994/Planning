#include "local_trajectory_combiner.h"

namespace Planning
{
  LocalTrajectoryCombiner::LocalTrajectoryCombiner()//轨迹合成器
  {
    RCLCPP_INFO(rclcpp::get_logger("local_trajectory_combiner"), "local_trajectory_combiner created");
  
    // 读取配置文件
    trajectory_config_ = std::make_unique<ConfigReader>();

  }

  // 合成轨迹
  LocalTrajectory LocalTrajectoryCombiner::combin_trajectory(const LocalPath &path, const LocalSpeeds &speeds)
  {
    const int path_size = path.local_path.size();
    const int speeds_size = speeds.local_speeds.size();
    local_trajectory_.header = path.header;
    local_trajectory_.local_trajectory.clear();

    // if(path_size < 3 || speeds_size < 3)
    if(path_size < 3)
    {
      RCLCPP_WARN(rclcpp::get_logger("trajectory"),"local path or speeds empty!");
      return local_trajectory_;
    }

    LocalTrajectoryPoint point_tmp;
    for(int i = 0; i<path_size; i++)
    {
      //路径部分的赋值
      point_tmp.path_point = path.local_path[i];

      //速度部分的赋值

      //填充进轨迹
      local_trajectory_.local_trajectory.emplace_back(point_tmp);
    }

    RCLCPP_INFO(rclcpp::get_logger("trajectory"),"trajectory combined, size = %ld",local_trajectory_.local_trajectory.size());
    return local_trajectory_;
  }
} // namespace Planning
