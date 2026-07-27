#ifndef LOCAL_PATH_PLANNER_H_
#define LOCAL_PATH_PLANNER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/local_path_point.hpp"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "polynomial_curve.h"
#include "main_car_info.h"
#include "decision_center.h"
#include "local_path_smoother.h"

namespace Planning
{
  using base_msgs::msg::LocalPath;
  using base_msgs::msg::LocalPathPoint;
  using base_msgs::msg::Referline;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;

  class LocalPathPlanner // 局部路径规划器
  {
  public:
    LocalPathPlanner();

    // 生成局部路径
    LocalPath creat_local_path(const Referline &reference_line, const std::shared_ptr<VehicleBase> &car,
                               const std::shared_ptr<DecisionCenter> &decision);
    Path path_to_rviz(); // 转换成rviz

  private:
    void init_local_path(); // 初始化局部路径

  public:
    inline Path local_path_rviz() const { return local_path_rviz_; } // 获取局部路径rviz用
    inline LocalPath local_path() const { return local_path_; }      // 获取局部路径

  private:
    std::unique_ptr<ConfigReader> local_path_config_;        // 配置
    Path local_path_rviz_;                                   // 局部路径给rviz用
    LocalPath local_path_;                                   // 局部路径
    std::shared_ptr<LocalPathSmoother> local_path_smoother_; // 局部路径平滑器
  };
} // namespace Planning
#endif // LOCAL_PATH_PLANNER_H_
