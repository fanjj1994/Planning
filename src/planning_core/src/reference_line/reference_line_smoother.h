#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace Planning {
  class ReferenceLineSmoother
  {
  public:
    ReferenceLineSmoother();
    ReferenceLineSmoother(const ReferenceLineSmoother &) = delete;
    ReferenceLineSmoother &operator=(const ReferenceLineSmoother &) = delete;
    ~ReferenceLineSmoother() = default;

  private:
  };
} // namespace Planning
#endif // ! REFERENCE_LINE_SMOOTHER_H_
