#ifndef REFERENCE_LINE_SMOOTHER_H_
#define REFERENCE_LINE_SMOOTHER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"

#include "config_reader.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <cmath>

namespace Planning
{
  class ReferenceLineSmoother
  {
  public:
    ReferenceLineSmoother();
    ReferenceLineSmoother(const ReferenceLineSmoother &) = delete;
    ReferenceLineSmoother &operator=(const ReferenceLineSmoother &) = delete;
    ~ReferenceLineSmoother() = default;
    void SmoothReferenceLine(base_msgs::msg::Referline &referline); // smooth the reference line

  private:
    std::unique_ptr<ConfigReader> referenceLineConfigReader;
    // weight for smoothness term
    const float64 w1 = 100.0;
    const float64 w2 = 10.0;
    const float64 w3 = 1.0;
  };
} // namespace Planning
#endif // ! REFERENCE_LINE_SMOOTHER_H_
