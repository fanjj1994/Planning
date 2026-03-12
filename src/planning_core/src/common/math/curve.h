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

  /// \brief Cartesian state of a point: position, heading, speed, acceleration, curvature
  struct CartesianState
  {
    float64 x{ 0.0 };            ///< x coordinate in global frame [m]
    float64 y{ 0.0 };            ///< y coordinate in global frame [m]
    float64 theta{ 0.0 };        ///< heading angle (velocity direction w.r.t. x-axis) [rad]
    float64 speed{ 0.0 };        ///< speed scalar [m/s]
    float64 acceleration{ 0.0 }; ///< acceleration scalar (dv/dt) [m/s^2]
    float64 curvature{ 0.0 };    ///< trajectory curvature [1/m]
  };

  /// \brief Frenet state: longitudinal/lateral offsets and their derivatives
  struct FrenetState
  {
    float64 s{ 0.0 };      ///< longitudinal arc-length along reference line [m]
    float64 ds_dt{ 0.0 };  ///< ds/dt, longitudinal velocity [m/s]
    float64 dds_dt{ 0.0 }; ///< d^2s/dt^2, longitudinal acceleration [m/s^2]
    float64 l{ 0.0 };      ///< lateral offset (left-positive) [m]
    float64 dl_ds{ 0.0 };  ///< dl/ds, lateral offset rate w.r.t. arc-length (l') [dimensionless]
    float64 dl_dt{ 0.0 };  ///< dl/dt, lateral velocity [m/s]
    float64 ddl_ds{ 0.0 }; ///< d^2l/ds^2, lateral offset second derivative (l'') [1/m]
    float64 ddl_dt{ 0.0 }; ///< d^2l/dt^2, lateral acceleration [m/s^2]
  };

  struct FrenetStateOnLocalPath
  {
    float64 s_2path{ 0.0 };      ///< longitudinal arc-length along local path [m]
    float64 ds_dt_2path{ 0.0 };  ///< ds/dt on local path, longitudinal velocity [m/s]
    float64 dds_dt_2path{ 0.0 }; ///< d^2s/dt^2 on local path, longitudinal acceleration [m/s^2]
    float64 l_2path{ 0.0 };      ///< lateral offset to local path (left-positive) [m]
    float64 dl_ds_2path{ 0.0 };  ///< dl/ds on local path, lateral offset rate w.r.t. arc-length (l') [dimensionless]
    float64 dl_dt_2path{ 0.0 };  ///< dl/dt on local path, lateral velocity [m/s]
    float64 ddl_ds_2path{ 0.0 }; ///< d^2l/ds^2 on local path, lateral offset second derivative (l'') [1/m]
    float64 ddl_dt_2path{ 0.0 }; ///< d^2l/dt^2 on local path, lateral acceleration [m/s^2]
  };

  /// \brief Reference line projected point parameters
  struct ProjectedPointInfo
  {
    float64 rs{ 0.0 };      ///< arc-length of the projected point on the reference line [m]
    float64 rx{ 0.0 };      ///< projected point x coordinate [m]
    float64 ry{ 0.0 };      ///< projected point y coordinate [m]
    float64 rtheta{ 0.0 };  ///< reference line heading at projected point [rad]
    float64 rkappa{ 0.0 };  ///< reference line curvature at projected point [1/m]
    float64 rdkappa{ 0.0 }; ///< derivative of curvature w.r.t. arc-length at projected point [1/m^2]
  };

  class Curve
  {
  private:
    static constexpr uint8 MAX_JUMP = 100U;    // max jump between two adjacent matchpoint index
    static constexpr float64 EPSILON = 1.0e-6; // a small value
    static constexpr float64 DELTASMIN = 1.0;  // threshold value to determine if rs and r is close enough

  public:
    Curve() = default;
    ~Curve() = default;

    /// \brief                    Normalize angle to [-pi, pi]
    ///
    /// \param[in]                angle : Input angle in radians
    ///
    /// \return                   float64: Normalized angle in radians in the range [-pi, pi]
    static float64 NormalizeAngle(const float64 angle);

    /// \brief  Convert Cartesian state to Frenet state on reference line.
    ///
    /// Given a point's Cartesian state (x, y, theta, v, a, kappa) and its projected point
    /// on the reference line, compute the corresponding Frenet state (s, l and derivatives) on reference line.
    ///
    /// \par Conversion Formulas (define A = 1 - kappa_r * l, delta_theta = theta - theta_r):
    ///   1. s  = r_s
    ///   2. l  = sign(t x d) * |d|,  where t = (cos(theta_r), sin(theta_r)), d = (x-rx, y-ry)
    ///   3. l' = A * tan(delta_theta)
    ///   4. l''= -kappa_l_prime * tan(delta_theta)
    ///           + A * delta_theta_prime / cos^2(delta_theta)
    ///      where kappa_l_prime   = kappa_r' * l + kappa_r * l'
    ///            delta_theta_prime = A * kappa / cos(delta_theta) - kappa_r
    ///   5. sdot  = v * cos(delta_theta) / A
    ///   6. sddot = (a * cos(delta_theta) - sdot^2 * (l' * delta_theta_prime - kappa_l_prime)) / A
    ///   7. ldot  = v * sin(delta_theta)
    ///   8. lddot = a * sin(delta_theta)   (approximate; exact: l'' * sdot^2 + l' * sddot)
    ///
    /// \par Solve Order:
    ///   s, l  ->  l'  ->  l''  ->  sdot  ->  sddot  ->  ldot  ->  lddot
    ///
    /// \param[in]  cartesian       Cartesian state of the point
    /// \param[in]  projectedPoint  Projected point info on the reference line
    /// \param[out] frenet          Computed Frenet state on reference line
    static void CartesianToFrenet(const CartesianState &cartesian, const ProjectedPointInfo &projectedPoint,
                                  FrenetState &frenet);

    /// \brief  Convert Cartesian state to Frenet state on local path.
    ///
    /// Given a point's Cartesian state (x, y, theta, v, a, kappa) and its projected point
    /// on the reference line, compute the corresponding Frenet state (s, l and derivatives) on local path.
    ///
    /// \par Conversion Formulas (define A = 1 - kappa_r * l, delta_theta = theta - theta_r):
    ///   1. s  = r_s
    ///   2. l  = sign(t x d) * |d|,  where t = (cos(theta_r), sin(theta_r)), d = (x-rx, y-ry)
    ///   3. l' = A * tan(delta_theta)
    ///   4. l''= -kappa_l_prime * tan(delta_theta)
    ///           + A * delta_theta_prime / cos^2(delta_theta)
    ///      where kappa_l_prime   = kappa_r' * l + kappa_r * l'
    ///            delta_theta_prime = A * kappa / cos(delta_theta) - kappa_r
    ///   5. sdot  = v * cos(delta_theta) / A
    ///   6. sddot = (a * cos(delta_theta) - sdot^2 * (l' * delta_theta_prime - kappa_l_prime)) / A
    ///   7. ldot  = v * sin(delta_theta)
    ///   8. lddot = a * sin(delta_theta)   (approximate; exact: l'' * sdot^2 + l' * sddot)
    ///
    /// \par Solve Order:
    ///   s, l  ->  l'  ->  l''  ->  sdot  ->  sddot  ->  ldot  ->  lddot
    ///
    /// \param[in]  cartesian       Cartesian state of the point
    /// \param[in]  projectedPoint  Projected point info on the local path
    /// \param[out] frenet          Computed Frenet state on local path
    static void CartesianToFrenet(const CartesianState& cartesian, const ProjectedPointInfo& projectedPoint,
                                FrenetStateOnLocalPath& frenet);

    /// \brief  Convert Frenet state to Cartesian state.
    ///
    /// Given a point's Frenet state (s, sdot, sddot, l, l', l'') and its projected point
    /// on the reference line, compute the corresponding Cartesian state (x, y, theta, v, a, kappa).
    ///
    /// \par Conversion Formulas (define A = 1 - kappa_r * l, delta_theta = atan2(l', A)):
    ///   1. x     = rx - l * sin(theta_r)
    ///   2. y     = ry + l * cos(theta_r)
    ///   3. theta = theta_r + atan2(l', A)
    ///   4. kappa = ((l'' + kappa_l_prime * tan(delta_theta)) * cos^2(delta_theta) / A + kappa_r)
    ///              * cos(delta_theta) / A
    ///      where kappa_l_prime = kappa_r' * l + kappa_r * l'
    ///   5. v     = sqrt((sdot * A)^2 + (sdot * l')^2)
    ///   6. a     = sddot * A / cos(delta_theta)
    ///              + sdot^2 / cos(delta_theta) * (l' * delta_theta_prime - kappa_l_prime)
    ///      where delta_theta_prime = A * kappa / cos(delta_theta) - kappa_r
    ///
    /// \par Solve Order:
    ///   (x, y)  ->  theta  ->  kappa  ->  v  ->  a
    ///
    /// \param[in]  frenet          Frenet state of the point
    /// \param[in]  projectedPoint  Projected point info on the reference line
    /// \param[out] cartesian       Computed Cartesian state
    static void FrenetToCartesian(const FrenetState &frenet, const ProjectedPointInfo &projectedPoint,
                                  CartesianState &cartesian);

    /// \brief  Find the index of the closest point (match point) on a nav_msgs::Path to the target point,
    ///         constrained by a maximum jump distance from the last known match point index.
    ///
    /// @startuml
    /// start
    /// :pathSize = path.poses.size();
    /// if (pathSize <= 1?) then (yes)
    ///   :return pathSize - 1;
    ///   stop
    /// endif
    /// :distanceMin = MAX, closestIndex = -1;
    /// while (i < pathSize?) is (yes)
    ///   :distance = hypot(path[i] - targetPoint);
    ///   if (distance < distanceMin?) then (yes)
    ///     if (|lastMatchPointIndex - i| > MAX_JUMP?) then (yes)
    ///       :skip (prevent jumping);
    ///     else (no)
    ///       :distanceMin = distance\nclosestIndex = i;
    ///     endif
    ///   endif
    /// endwhile (no)
    /// :return closestIndex;
    /// stop
    /// @enduml
    ///
    /// \param[in]  path                 Global path (nav_msgs::Path)
    /// \param[in]  lastMatchPointIndex  Index of the previous match point (used to limit search range)
    /// \param[in]  targetPoint          The target point to project
    ///
    /// \return     int16: Index of the closest path point, or -1 if not found
    static int16 findMatchPointIndex(const nav_msgs::msg::Path &path, const int16 lastMatchPointIndex,
                                     const geometry_msgs::msg::PoseStamped &targetPoint);

    /// \brief  Find the index of the closest point (match point) on a Referline to the target point.
    ///         No jump constraint is applied (searches all points).
    ///
    /// @startuml
    /// start
    /// :pathSize = referenceline.refer_line.size();
    /// if (pathSize <= 1?) then (yes)
    ///   :return pathSize - 1;
    ///   stop
    /// endif
    /// :distanceMin = MAX, closestIndex = -1;
    /// while (i < pathSize?) is (yes)
    ///   :distance = hypot(refer_line[i] - targetPoint);
    ///   if (distance < distanceMin?) then (yes)
    ///     :distanceMin = distance\nclosestIndex = i;
    ///   endif
    /// endwhile (no)
    /// :return closestIndex;
    /// stop
    /// @enduml
    ///
    /// \param[in]  referenceline  Reference line (base_msgs::Referline)
    /// \param[in]  targetPoint    The target point to project
    ///
    /// \return     int16: Index of the closest reference line point, or -1 if not found
    static int16 findMatchPointIndex(const base_msgs::msg::Referline &referenceline,
                                     const geometry_msgs::msg::PoseStamped &targetPoint);

    /// \brief  Find the index of the closest point (match point) on a Referline to a given arc-length \c rs.
    ///         No jump constraint is applied (searches all points).
    ///
    /// \details
    /// This overload searches in the 1D Frenet longitudinal coordinate: it selects the point whose stored
    /// \c refer_line[i].rs has the smallest absolute difference to the query \c rs.
    ///
    /// \note The \c rs field of each reference line point must be pre-populated (e.g., by
    ///       calculateProjectedPointParameters(base_msgs::msg::Referline&)).
    ///
    /// @startuml
    /// start
    /// :pathSize = referenceline.refer_line.size();
    /// if (pathSize <= 1?) then (yes)
    ///   :return pathSize - 1;
    ///   stop
    /// endif
    /// :minAbsDiff = MAX, closestIndex = -1;
    /// while (i < pathSize?) is (yes)
    ///   :absDiff = abs(refer_line[i].rs - rs);
    ///   if (absDiff < minAbsDiff?) then (yes)
    ///     :minAbsDiff = absDiff\nclosestIndex = i;
    ///   endif
    /// endwhile (no)
    /// :return closestIndex;
    /// stop
    /// @enduml
    ///
    /// \param[in]  referenceline  Reference line (base_msgs::Referline)
    /// \param[in]  rs            Query arc-length along the reference line [m]
    ///
    /// \return     int16: Index of the reference line point with \c rs closest to the query (for out-of-range queries,
    ///             this returns the nearest endpoint). Returns -1 when the reference line is empty.
    static int16 findMatchPointIndex(const base_msgs::msg::Referline &referenceline, const float64 rs);

    /// \brief  Find the index of the closest point (match point) on a LocalPath to the target point.
    ///         No jump constraint is applied (searches all points).
    ///
    /// \details
    /// LocalPath stores points in \c localPath.local_path. This overload mirrors the Referline version, but operates on
    /// the local-planner output message type.
    ///
    /// @startuml
    /// start
    /// :pathSize = localPath.local_path.size();
    /// if (pathSize <= 1?) then (yes)
    ///   :return pathSize - 1;
    ///   stop
    /// endif
    /// :distanceMin = MAX, closestIndex = -1;
    /// while (i < pathSize?) is (yes)
    ///   :distance = hypot(local_path[i] - targetPoint);
    ///   if (distance < distanceMin?) then (yes)
    ///     :distanceMin = distance\nclosestIndex = i;
    ///   endif
    /// endwhile (no)
    /// :return closestIndex;
    /// stop
    /// @enduml
    ///
    /// \param[in]  localPath     Local path (base_msgs::LocalPath)
    /// \param[in]  targetPoint   The target point to project
    ///
    /// \return     int16: Index of the closest local path point, or -1 if not found (e.g., empty path)
    static int16 findMatchPointIndex(const base_msgs::msg::LocalPath &localPath,
                                     const geometry_msgs::msg::PoseStamped &targetPoint);

    /// \brief  Determine the projected point (closest reference line point) for a given target point,
    ///         and populate its geometric parameters (rs, rx, ry, rtheta, rkappa, rdkappa).
    ///
    /// @startuml
    /// start
    /// :matchPointIndex = findMatchPointIndex(referenceline, targetPoint);
    /// if (matchPointIndex >= 0?) then (yes)
    ///   :projectedPoint.rs     = refer_line[idx].rs;
    ///   :projectedPoint.rx     = refer_line[idx].x;
    ///   :projectedPoint.ry     = refer_line[idx].y;
    ///   :projectedPoint.rtheta = refer_line[idx].rtheta;
    ///   :projectedPoint.rkappa = refer_line[idx].rkappa;
    ///   :projectedPoint.rdkappa= refer_line[idx].rdkappa;
    /// else (no)
    ///   :LOG error "Failed to find projected point";
    ///   stop
    /// endif
    /// stop
    /// @enduml
    ///
    /// \param[in]  referenceline   Reference line (base_msgs::Referline)
    /// \param[in]  targetPoint     The target point to project
    /// \param[out] projectedPoint  Output projected point info populated from the closest reference line point
    static void figureOutProjectedPoint(const base_msgs::msg::Referline &referenceline,
                                        const geometry_msgs::msg::PoseStamped &targetPoint,
                                        ProjectedPointInfo &projectedPoint);

    /// \brief  Determine the projected point (closest local path point) for a given target point,
    ///         and populate its geometric parameters (rs, rx, ry, rtheta, rkappa, rdkappa).
    ///
    /// \details
    /// This is a simplified projection: it selects the closest discrete point on \c localPath.local_path (no segment
    /// interpolation). The output fields are filled from that LocalPathPoint:
    /// - \c rs, \c rx, \c ry come from the matched point's accumulated arc-length and position.
    /// - \c rtheta, \c rkappa, \c rdkappa come from the matched point's pre-computed reference parameters.
    ///
    /// @startuml
    /// start
    /// :matchPointIndex = findMatchPointIndex(localPath, targetPoint);
    /// if (matchPointIndex >= 0?) then (yes)
    ///   :projectedPoint.rs      = local_path[idx].rs;
    ///   :projectedPoint.rx      = local_path[idx].x;
    ///   :projectedPoint.ry      = local_path[idx].y;
    ///   :projectedPoint.rtheta  = local_path[idx].rtheta;
    ///   :projectedPoint.rkappa  = local_path[idx].rkappa;
    ///   :projectedPoint.rdkappa = local_path[idx].rdkappa;
    /// else (no)
    ///   :LOG error "Failed to find projected point";
    ///   stop
    /// endif
    /// stop
    /// @enduml
    ///
    /// \param[in]  localPath       Local path (base_msgs::LocalPath)
    /// \param[in]  targetPoint     The target point to project
    /// \param[out] projectedPoint  Output projected point info populated from the closest local path point
    static void figureOutProjectedPoint(const base_msgs::msg::LocalPath &localPath,
                                        const geometry_msgs::msg::PoseStamped &targetPoint,
                                        ProjectedPointInfo &projectedPoint);

    /// \brief  Calculate and populate the geometric parameters (rs, rtheta, rkappa, rdkappa)
    ///         for every point on the reference line using finite-difference approximation.
    ///
    /// @startuml
    /// start
    /// :pathSize = referenceline.refer_line.size();
    /// if (pathSize < 3?) then (yes)
    ///   :LOG error "size < 3";
    ///   stop
    /// endif
    ///
    /// partition "Step 1: Compute arc-length rs" {
    ///   :rs[0] = 0;
    ///   while (i = 1 .. pathSize-1) is (yes)
    ///     :dx = x[i] - x[i-1];
    ///     :dy = y[i] - y[i-1];
    ///     :segmentLen = sqrt(dx*dx + dy*dy);  (Euclidean distance between adjacent points)
    ///     :rs[i] = rs[i-1] + segmentLen;
    ///   endwhile (done)
    /// }
    ///
    /// partition "Step 2: Compute heading rtheta" {
    ///   while (i = 0 .. pathSize-1) is (yes)
    ///     if (i < pathSize-1?) then (forward diff)
    ///       :rtheta[i] = atan2(y[i+1]-y[i], x[i+1]-x[i]);
    ///     else (backward diff)
    ///       :rtheta[i] = atan2(y[i]-y[i-1], x[i]-x[i-1]);
    ///     endif
    ///   endwhile (done)
    /// }
    ///
    /// partition "Step 3: Compute curvature rkappa = dtheta/ds" {
    ///   while (i = 0 .. pathSize-1) is (yes)
    ///     :segmentLen = distance between two adjacent points (sqrt((x2-x1)^2 + (y2-y1)^2));
    ///     if (segmentLen <= EPSILON?) then (yes)
    ///       :rkappa[i] = 0;
    ///     else (no)
    ///       :rkappa[i] = (rtheta[i+1] - rtheta[i]) / segmentLen;
    ///       note right: last point uses backward diff
    ///     endif
    ///   endwhile (done)
    /// }
    ///
    /// partition "Step 4: Compute curvature derivative rdkappa = dkappa/ds" {
    ///   while (i = 0 .. pathSize-1) is (yes)
    ///     :segmentLen = distance between two adjacent points (sqrt((x2-x1)^2 + (y2-y1)^2));
    ///     if (segmentLen <= EPSILON?) then (yes)
    ///       :rdkappa[i] = 0;
    ///     else (no)
    ///       :rdkappa[i] = (rkappa[i+1] - rkappa[i]) / segmentLen;
    ///       note right: last point uses backward diff
    ///     endif
    ///   endwhile (done)
    /// }
    /// stop
    /// @enduml
    ///
    /// \param[in,out] referenceline  Reference line whose points will have rs, rtheta, rkappa, rdkappa populated
    static void calculateProjectedPointParameters(base_msgs::msg::Referline &referenceline);

    /// \brief  Calculate and populate the geometric parameters (rs, rtheta, rkappa, rdkappa)
    ///         for every point on a LocalPath.
    ///
    /// \details
    /// This overload mirrors the Referline version but differs in where heading/curvature come from:
    /// - \c rs is accumulated from the \c localPath.local_path positions (finite differences).
    /// - \c rtheta and \c rkappa are copied directly from each LocalPathPoint's \c theta and \c kappa fields (the local
    ///   path is assumed to already carry these values), rather than re-estimating from positions.
    /// - \c rdkappa is then computed via finite differences with respect to arc-length.
    ///
    /// \note Requires at least 3 points to compute a stable derivative; otherwise the function logs an error and
    ///       returns without modifying the path.
    ///
    /// @startuml
    /// start
    /// :pathSize = localPath.local_path.size();
    /// if (pathSize < 3?) then (yes)
    ///   :LOG error "size < 3";
    ///   stop
    /// endif
    ///
    /// partition "Step 1: Compute arc-length rs" {
    ///   :rs[0] = 0;
    ///   while (i = 1 .. pathSize-1) is (yes)
    ///     :dx = x[i] - x[i-1];
    ///     :dy = y[i] - y[i-1];
    ///     :segmentLen = sqrt(dx*dx + dy*dy);  (Euclidean distance between adjacent points)
    ///     :rs[i] = rs[i-1] + segmentLen;
    ///   endwhile (done)
    /// }
    ///
    /// partition "Step 2: Copy heading/curvature" {
    ///   while (i = 0 .. pathSize-1) is (yes)
    ///     :rtheta[i] = theta[i];
    ///     :rkappa[i] = kappa[i];
    ///   endwhile (done)
    /// }
    ///
    /// partition "Step 3: Compute curvature derivative rdkappa = dkappa/ds" {
    ///   while (i = 0 .. pathSize-1) is (yes)
    ///     :segmentLen = distance between two adjacent points (sqrt((x2-x1)^2 + (y2-y1)^2));
    ///     if (segmentLen <= EPSILON?) then (yes)
    ///       :rdkappa[i] = 0;
    ///     else (no)
    ///       :rdkappa[i] = (rkappa[i+1] - rkappa[i]) / segmentLen;
    ///       note right: last point uses backward diff
    ///     endif
    ///   endwhile (done)
    /// }
    /// stop
    /// @enduml
    ///
    /// \param[in,out] localPath  Local path whose points will have rs, rtheta, rkappa, rdkappa populated
    static void calculateProjectedPointParameters(base_msgs::msg::LocalPath &localPath);
  };
} // namespace Planning
#endif // CURVE_H_
