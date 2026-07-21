#ifndef REFERENCE_LINE_CREATOR_H_
#define REFERENCE_LINE_CREATOR_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/referline_point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>

#include "config_reader.h"
#include "curve.h"
#include "reference_line_smoother.h"

namespace Planning
{
  using base_msgs::msg::Referline;
  using base_msgs::msg::ReferlinePoint;
  using geometry_msgs::msg::PoseStamped;
  using nav_msgs::msg::Path;

  class ReferencelineCreator // 创建参考线
  {
  public:
    ReferencelineCreator();

    Referline creat_reference_line(const Path &global_path,
                                   const PoseStamped &target_point); // 生成参考线
    Path referline_to_rviz();                                        // 转换成rviz的Path格式

  public:
    inline Referline reference_line() const { return refer_line_; }      // 获取参考线
    inline Path reference_line_rviz() const { return refer_line_rviz_; } // 获取参考线 for rviz
    inline int match_point_index() const { return match_point_index_; }  // 获取匹配点
    inline int front_index() const { return front_index_; }              // 获取最前点下标
    inline int back_index() const { return back_index_; }                // 获取最后点下标

  private:
    std::unique_ptr<ConfigReader> reference_line_config_;        // 配置
    Referline refer_line_;                                       // 参考线
    Path refer_line_rviz_;                                       // 参考线 for rviz
    std::shared_ptr<ReferencelineSmoother> refer_line_smoother_; // 参考线平滑器
    int last_match_index_ = -1;                                  // 上一帧匹配点在全局路径下的下标
    int match_point_index_ = -1;                                 // 匹配点在全局路径下的下标
    int front_index_ = -1;                                       // 最前点在全局路径下的下标
    int back_index_ = -1;                                        // 最后点在全局路径下的下标
  };
} // namespace Planning
#endif // REFERENCE_LINE_CREATOR_H_
