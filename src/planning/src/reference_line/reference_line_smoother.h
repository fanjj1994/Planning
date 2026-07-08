#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include <Eigen/Dense> //eigen
#include <OsqpEigen/OsqpEigen.h>//osqp-eigen
#include <cmath>

namespace Planning
{
  class ReferencelineSmoother // 创建参考线平滑器
  {
  public:
    ReferencelineSmoother();

  private:
  };
} // namespace Planning
#endif // REFERENCE_LINE_SMOOTHER_H_
