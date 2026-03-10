#include "curve.h"

namespace Planning
{
  float64 Curve::NormalizeAngle(const float64 angle)
  {
    // angle mod 2*pi to get the equivalent angle in range (-2*pi, 2*pi)
    float64 normalizedAngle = std::fmod(angle, 2.0 * M_PI); // wrap angle to (-2*pi, 2*pi)
    if (normalizedAngle > M_PI)
    {
      normalizedAngle -= 2.0 * M_PI;
    }
    else if (normalizedAngle < -M_PI)
    {
      normalizedAngle += 2.0 * M_PI;
    }
    return normalizedAngle;
  }

  void Curve::CartesianToFrenet(const CartesianState& cartesian, const ProjectedPointInfo& projectedPoint,
                                FrenetState& frenet)
  {
    // ================1. calculate longitudinal offset s================
    frenet.s = projectedPoint.rs;

    // ================2. calculate lateral offset l================
    const float64 dx = cartesian.x - projectedPoint.rx; // vector from projected point to target point in x direction
    const float64 dy = cartesian.y - projectedPoint.ry; // vector from projected point to target point in y direction

    const float64 cos_rtheta = std::cos(projectedPoint.rtheta);
    const float64 sin_rtheta = std::sin(projectedPoint.rtheta);

    // cross product of r's tangent vector and (x-r)
    const float64 tangentCrossDisp = -sin_rtheta * dx + cos_rtheta * dy;
    frenet.l = std::copysign(std::hypot(dx, dy), tangentCrossDisp); // lateral offset with sign

    const float64 A = 1.0 - projectedPoint.rkappa * frenet.l; // A = 1 - kappa_r * l

    // ================3. calculate l' = dl/ds================
    // heading difference between target point and projected point
    const float64 delta_theta = NormalizeAngle(cartesian.theta - projectedPoint.rtheta);
    const float64 tan_delta_theta = std::tan(delta_theta);
    const float64 sin_delta_theta = std::sin(delta_theta);
    const float64 cos_delta_theta = std::cos(delta_theta);
    frenet.dl_ds = A * tan_delta_theta; // dl/ds = A * tan(delta_theta)

    //================4. calculate l'' = d(dl/ds)/ds================
    // kappa_r' * l + kappa_r * l'
    const float64 kappa_l_prime = projectedPoint.rdkappa * frenet.l + projectedPoint.rkappa * frenet.dl_ds;
    // delta_theta' = A * curvature / cos(delta_theta) - kappa_r
    const float64 delta_theta_prime = A * cartesian.curvature / cos_delta_theta - projectedPoint.rkappa;

    frenet.ddl_ds = -kappa_l_prime * tan_delta_theta + A * delta_theta_prime / (cos_delta_theta * cos_delta_theta);

    //================5. calculate sdot = ds/dt================
    frenet.ds_dt = cartesian.speed * cos_delta_theta / A;

    //================6. calculate sddot = d^2s/dt^2================
    frenet.dds_dt = (cartesian.acceleration * cos_delta_theta -
                     (frenet.ds_dt * frenet.ds_dt) * (frenet.dl_ds * delta_theta_prime - kappa_l_prime)) /
                    A;

    //================7. calculate ldot = dl/dt================
    frenet.dl_dt = cartesian.speed * sin_delta_theta;

    //================8. calculate lddot = d^2l/dt^2================
    frenet.ddl_dt = cartesian.acceleration * sin_delta_theta;
  }

  void Curve::FrenetToCartesian(const FrenetState& frenet, const ProjectedPointInfo& projectedPoint,
                                CartesianState& cartesian)
  {
    // identify the distance between s and rs, which is the projected point's s on reference line
    if (std::fabs(projectedPoint.rs - frenet.s) > DELTASMIN)
    {
      RCLCPP_ERROR(rclcpp::get_logger("math"), "reference pt s and projection rs not match, rs: %.2f, s: %.2f",
                   projectedPoint.rs, frenet.s);
      return;
    }

    // ================1. calculate Cartesian coordinate (x, y)================
    const float64 cos_rtheta = std::cos(projectedPoint.rtheta);
    const float64 sin_rtheta = std::sin(projectedPoint.rtheta);
    cartesian.x = projectedPoint.rx - frenet.l * sin_rtheta; // x = rx - l * sin(theta_r)
    cartesian.y = projectedPoint.ry + frenet.l * cos_rtheta; // y = ry + l * cos(theta_r)

    // ================2. calculate heading theta================
    const float64 A = 1.0 - projectedPoint.rkappa * frenet.l; // A = 1 - kappa_r * l
    const float64 delta_theta = std::atan2(frenet.dl_ds, A);  // delta_theta = atan2(l', A)
    const float64 tan_delta_theta = frenet.dl_ds / A;         // tan(delta_theta) = l' / A
    const float64 cos_delta_theta = std::cos(delta_theta);
    cartesian.theta = NormalizeAngle(projectedPoint.rtheta + delta_theta); // theta = theta_r + delta_theta

    // ================3. calculate curvature================
    // kappa_r' * l + kappa_r * l'
    const float64 kappa_l_prime = projectedPoint.rdkappa * frenet.l + projectedPoint.rkappa * frenet.dl_ds;
    cartesian.curvature = ((frenet.ddl_ds + kappa_l_prime * tan_delta_theta) * (cos_delta_theta * cos_delta_theta) / A +
                           projectedPoint.rkappa) *
                          cos_delta_theta / A;

    // ================4. calculate speed================
    // speed = sqrt((sdot * A)^2 + (sdot * l')^2)
    cartesian.speed = std::hypot(frenet.ds_dt * A, frenet.ds_dt * frenet.dl_ds);

    // ================5. calculate acceleration================
    const float64 delta_theta_prime = A * cartesian.curvature / cos_delta_theta - projectedPoint.rkappa;
    cartesian.acceleration =
        frenet.dds_dt * A / cos_delta_theta +
        frenet.ds_dt * frenet.ds_dt / cos_delta_theta * (frenet.dl_ds * delta_theta_prime - kappa_l_prime);
  }

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
        if (abs(lastMatchPointIndex - i) > MAX_JUMP) // prevent jumping
        {
          continue;
        }
        distanceMin = distance;
        closestIndex = i;
      }
    }
    return closestIndex;
  }

  int16 Curve::findMatchPointIndex(const base_msgs::msg::Referline& referenceline,
                                   const geometry_msgs::msg::PoseStamped& targetPoint)
  {
    const int16 pathSize = referenceline.refer_line.size();
    if (pathSize <= 1)
    {
      return pathSize - 1;
    }
    float64 distanceMin = std::numeric_limits<float64>::max();
    int16 closestIndex = -1;

    for (int16 i = 0; i < pathSize; i++)
    {
      float64 distance = std::hypot(referenceline.refer_line[i].pose.pose.position.x - targetPoint.pose.position.x,
                                    referenceline.refer_line[i].pose.pose.position.y - targetPoint.pose.position.y);
      if (distance < distanceMin)
      {
        distanceMin = distance;
        closestIndex = i;
      }
    }
    return closestIndex;
  }

  int16 Curve::findMatchPointIndex(const base_msgs::msg::Referline& referenceline, const float64 rs)
  {
    const int16 pathSize = referenceline.refer_line.size();
    if (pathSize <= 1)
    {
      return pathSize - 1;
    }
    float64 distanceMin = std::numeric_limits<float64>::max();
    int16 closestIndex = -1;
    for (int16 i = 0; i < pathSize; i++)
    {
      float64 distance = std::abs(referenceline.refer_line[i].rs - rs);
      if (distance < distanceMin)
      {
        distanceMin = distance;
        closestIndex = i;
      }
    }
    return closestIndex;
  }

  int16 Curve::findMatchPointIndex(const base_msgs::msg::LocalPath& localPath,
                                   const geometry_msgs::msg::PoseStamped& targetPoint)
  {
    const int16 pathSize = localPath.local_path.size();
    if (pathSize <= 1)
    {
      return pathSize - 1;
    }
    float64 distanceMin = std::numeric_limits<float64>::max();
    int16 closestIndex = -1;

    for (int16 i = 0; i < pathSize; i++)
    {
      float64 distance = std::hypot(localPath.local_path[i].pose.pose.position.x - targetPoint.pose.position.x,
                                    localPath.local_path[i].pose.pose.position.y - targetPoint.pose.position.y);
      if (distance < distanceMin)
      {
        distanceMin = distance;
        closestIndex = i;
      }
    }
    return closestIndex;
  }

  void Curve::figureOutProjectedPoint(const base_msgs::msg::Referline& referenceline,
                                      const geometry_msgs::msg::PoseStamped& targetPoint,
                                      ProjectedPointInfo& projectedPoint)
  {
    // simplified: use the closest point on referenceline as projected point because the referenceline is dense enough
    const int16 matchPointIndex = findMatchPointIndex(referenceline, targetPoint);
    if (matchPointIndex >= 0)
    {
      projectedPoint.rs = referenceline.refer_line[matchPointIndex].rs;
      projectedPoint.rx = referenceline.refer_line[matchPointIndex].pose.pose.position.x;
      projectedPoint.ry = referenceline.refer_line[matchPointIndex].pose.pose.position.y;
      projectedPoint.rtheta = referenceline.refer_line[matchPointIndex].rtheta;
      projectedPoint.rkappa = referenceline.refer_line[matchPointIndex].rkappa;
      projectedPoint.rdkappa = referenceline.refer_line[matchPointIndex].rdkappa;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("math"), "Failed to find projected point on reference line for target point!");
      return;
    }
  }

  void Curve::figureOutProjectedPoint(const base_msgs::msg::LocalPath& localPath,
                                      const geometry_msgs::msg::PoseStamped& targetPoint,
                                      ProjectedPointInfo& projectedPoint)
  {
    // simplified: use the closest point on localPath as projected point because the localPath is dense enough
    const int16 matchPointIndex = findMatchPointIndex(localPath, targetPoint);
    if (matchPointIndex >= 0)
    {
      projectedPoint.rs = localPath.local_path[matchPointIndex].rs;
      projectedPoint.rx = localPath.local_path[matchPointIndex].pose.pose.position.x;
      projectedPoint.ry = localPath.local_path[matchPointIndex].pose.pose.position.y;
      projectedPoint.rtheta = localPath.local_path[matchPointIndex].rtheta;
      projectedPoint.rkappa = localPath.local_path[matchPointIndex].rkappa;
      projectedPoint.rdkappa = localPath.local_path[matchPointIndex].rdkappa;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("math"), "Failed to find projected point on local path for target point!");
      return;
    }
  }

  // calculate projected point parameters on reference line
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
      if (i < pathSize - 1U) // not the last point
      {
        referenceline.refer_line[i].rtheta = std::atan2(
            referenceline.refer_line[i + 1U].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
            referenceline.refer_line[i + 1U].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
      }
      else // the last point
      {
        referenceline.refer_line[i].rtheta = std::atan2(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
      }
    }

    // calculate kappa
    for (uint16 i = 0U; i < pathSize; i++)
    {
      if (i < pathSize - 1U) // not the last point
      {
        const float64 distance = std::hypot(
            referenceline.refer_line[i + 1U].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
            referenceline.refer_line[i + 1U].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
        if (distance <= EPSILON)
        {
          referenceline.refer_line[i].rkappa = 0.0; // prevent division by zero
        }
        else
        {
          // use kappa's definition: kappa = dtheta / ds
          referenceline.refer_line[i].rkappa =
              (referenceline.refer_line[i + 1U].rtheta - referenceline.refer_line[i].rtheta) / distance;
        }
      }
      else // the last point
      {
        const float64 distance = std::hypot(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
        if (distance <= EPSILON)
        {
          referenceline.refer_line[i].rkappa = 0.0; // prevent division by zero
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
      if (i < pathSize - 1U) // not the last point
      {
        const float64 distance = std::hypot(
            referenceline.refer_line[i + 1U].pose.pose.position.y - referenceline.refer_line[i].pose.pose.position.y,
            referenceline.refer_line[i + 1U].pose.pose.position.x - referenceline.refer_line[i].pose.pose.position.x);
        if (distance <= EPSILON)
        {
          referenceline.refer_line[i].rdkappa = 0.0; // prevent division by zero
        }
        else
        {
          // use dkappa's definition: dkappa / ds
          referenceline.refer_line[i].rdkappa =
              (referenceline.refer_line[i + 1U].rkappa - referenceline.refer_line[i].rkappa) / distance;
        }
      }
      else // the last point
      {
        const float64 distance = std::hypot(
            referenceline.refer_line[i].pose.pose.position.y - referenceline.refer_line[i - 1U].pose.pose.position.y,
            referenceline.refer_line[i].pose.pose.position.x - referenceline.refer_line[i - 1U].pose.pose.position.x);
        if (distance <= EPSILON)
        {
          referenceline.refer_line[i].rdkappa = 0.0; // prevent division by zero
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

  // calculate projected point parameters on local path
  void Curve::calculateProjectedPointParameters(base_msgs::msg::LocalPath& localPath)
  {
    const uint16 pathSize = localPath.local_path.size();
    if (pathSize < 3U)
    {
      RCLCPP_ERROR(rclcpp::get_logger("math"),
                   "Local path size is less than 3, cannot calculate projected point parameters!");
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
            localPath.local_path[i].pose.pose.position.y - localPath.local_path[i - 1U].pose.pose.position.y,
            localPath.local_path[i].pose.pose.position.x - localPath.local_path[i - 1U].pose.pose.position.x);
      }
      localPath.local_path[i].rs = rs;
    }

    // give the value to heading and curvature (kappa)
    for (uint16 i = 0U; i < pathSize; i++)
    {
      localPath.local_path[i].rtheta = localPath.local_path[i].theta; // use the heading in local path directly
      localPath.local_path[i].rkappa = localPath.local_path[i].kappa; // use the curvature in local path directly
    }

    // calculate dkappa
    for (uint16 i = 0U; i < pathSize; i++)
    {
      if (i < pathSize - 1U) // not the last point
      {
        const float64 distance = std::hypot(
            localPath.local_path[i + 1U].pose.pose.position.y - localPath.local_path[i].pose.pose.position.y,
            localPath.local_path[i + 1U].pose.pose.position.x - localPath.local_path[i].pose.pose.position.x);
        if (distance <= EPSILON)
        {
          localPath.local_path[i].rdkappa = 0.0; // prevent division by zero
        }
        else
        {
          // use dkappa's definition: dkappa / ds
          localPath.local_path[i].rdkappa =
              (localPath.local_path[i + 1U].rkappa - localPath.local_path[i].rkappa) / distance;
        }
      }
      else // the last point
      {
        const float64 distance = std::hypot(
            localPath.local_path[i].pose.pose.position.y - localPath.local_path[i - 1U].pose.pose.position.y,
            localPath.local_path[i].pose.pose.position.x - localPath.local_path[i - 1U].pose.pose.position.x);
        if (distance <= EPSILON)
        {
          localPath.local_path[i].rdkappa = 0.0; // prevent division by zero
        }
        else
        {
          // use dkappa's definition: dkappa / ds
          localPath.local_path[i].rdkappa =
              (localPath.local_path[i].rkappa - localPath.local_path[i - 1U].rkappa) / distance;
        }
      }
    }
  }

} // namespace Planning
