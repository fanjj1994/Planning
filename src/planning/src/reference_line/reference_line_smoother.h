#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include <Eigen/Dense> //eigen
#include <OsqpEigen/OsqpEigen.h>//osqp-eigen
#include <cmath>

#include "config_reader.h"

namespace Planning
{
  using base_msgs::msg::Referline;
  class ReferencelineSmoother // 创建参考线平滑器
  {
  public:
    ReferencelineSmoother();
    void smooth_reference_line(Referline &refer_line); // 平滑参考线

  private:
    std::unique_ptr<ConfigReader> reference_line_config_;// 配置
    const double w1 = 100.0;
    const double w2 = 10.0;
    const double w3 = 1.0;
  };
} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_
