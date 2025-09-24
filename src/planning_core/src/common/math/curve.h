#ifndef CURVE_H_
#define CURVE_H_

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "common_type.h"
#include "base_msgs/msg/local_path.hpp"
#include "base_msgs/msg/referline.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace Planning
{
  class Curve
  {
  private:
    static constexpr uint8 MAX_JUMP = 100U;    // max jump between two adjacent matchpoint index
    static constexpr float64 EPSILON = 1.0e-6; // a small value

  public:
    Curve() = default;
    ~Curve() = default;

    // find matchpoint index
    static int16 findMatchPointIndex(const nav_msgs::msg::Path& path, const int16 lastMatchPointIndex,
                                     const geometry_msgs::msg::PoseStamped& targetPoint);
    // find projected point's parameters
    static void calculateProjectedPointParameters(base_msgs::msg::Referline& referenceline);
  };
} // namespace Planning
#endif // CURVE_H_
