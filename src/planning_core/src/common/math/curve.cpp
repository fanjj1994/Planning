#include "curve.h"

namespace Planning
{
int16 Curve::findMatchPointIndex(const nav_msgs::msg::Path& path, const int16 lastMatchPointIndex,
                                 const geometry_msgs::msg::PoseStamped& targetPoint)
{
  const int16 pathSize = path.poses.size();
  if (pathSize <= 1)
  {
    return pathSize - 1;
  }
  float64 distanceMin = std::numeric_limits<float64>::max();
  int16 closestIndex = -1;

  for (int16 i = 0; i < pathSize; i++)
  {
    float64 distance = std::hypot(path.poses[i].pose.position.x - targetPoint.pose.position.x,
                                  path.poses[i].pose.position.y - targetPoint.pose.position.y);
    if (distance < distanceMin)
    {
      if (abs(lastMatchPointIndex - i) > MAX_JUMP)  // prevent jumping
      {
        continue;
      }
      distanceMin = distance;
      closestIndex = i;
    }
  }
  return closestIndex;
}

void Curve::calculateProjectedPointParameters(base_msgs::msg::Referline& referenceline)
{
  const uint16 pathSize = referenceline.refer_line.size();
  if (pathSize < 3U)
  {
    RCLCPP_ERROR(rclcpp::get_logger("math"),
                 "Referline size is less than 3, cannot calculate projected point parameters!");
    return;
  }

  // calculate rs
  float64 rs = 0.0;
  for (uint16 i = 0U; i < pathSize; i++)
  {
    if (i == 0U)
    {
      rs = 0.0;
    }
    else
    {
      // std::hypot is similar with sqrt(x^2 + y^2), which calculates two points' distance safely without overflow
      rs += std::hypot(
          referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
          referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
    }
    referenceline.refer_line[i].rs = rs;
  }

  // calculate heading
  for (uint16 i = 0U; i < pathSize; i++)
  {
    if (i < pathSize - 1U)  // not the last point
    {
      referenceline.refer_line[i].rtheta = std::atan2(
          referenceline.refer_line[i + 1U].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
          referenceline.refer_line[i + 1U].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
    }
    else  // the last point
    {
      referenceline.refer_line[i].rtheta = std::atan2(
          referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
          referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
    }
  }

  // calculate kappa
  for (uint16 i = 0U; i < pathSize; i++)
  {
    if (i < pathSize - 1U)  // not the last point
    {
      const float64 distance = std::hypot(
          referenceline.refer_line[i + 1U].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
          referenceline.refer_line[i + 1U].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
      if (distance <= EPSILON)
      {
        referenceline.refer_line[i].rkappa = 0.0;  // prevent division by zero
      }
      else
      {
        // use kappa's definition: kappa = dtheta / ds
        referenceline.refer_line[i].rkappa =
            (referenceline.refer_line[i + 1U].rtheta - referenceline.refer_line[i].rtheta) / distance;
      }
    }
    else  // the last point
    {
      const float64 distance = std::hypot(
          referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
          referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
      if (distance <= EPSILON)
      {
        referenceline.refer_line[i].rkappa = 0.0;  // prevent division by zero
      }
      else
      {
        // use kappa's definition: kappa = dtheta / ds
        referenceline.refer_line[i].rkappa =
            (referenceline.refer_line[i].rtheta - referenceline.refer_line[i - 1U].rtheta) / distance;
      }
    }
  }

  // calculate dkappa
  for (uint16 i = 0U; i < pathSize; i++)
  {
    if (i < pathSize - 1U)  // not the last point
    {
      const float64 distance = std::hypot(
          referenceline.refer_line[i + 1U].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
          referenceline.refer_line[i + 1U].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
      if (distance <= EPSILON)
      {
        referenceline.refer_line[i].rdkappa = 0.0;  // prevent division by zero
      }
      else
      {
        // use dkappa's definition: dkappa / ds
        referenceline.refer_line[i].rdkappa =
            (referenceline.refer_line[i + 1U].rkappa - referenceline.refer_line[i].rkappa) / distance;
      }
    }
    else  // the last point
    {
      const float64 distance = std::hypot(
          referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
          referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
      if (distance <= EPSILON)
      {
        referenceline.refer_line[i].rdkappa = 0.0;  // prevent division by zero
      }
      else
      {
        // use dkappa's definition: dkappa / ds
        referenceline.refer_line[i].rdkappa =
            (referenceline.refer_line[i].rkappa - referenceline.refer_line[i - 1U].rkappa) / distance;
      }
    }
  }
}

}  // namespace Planning
