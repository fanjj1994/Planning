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
#include "ego_car_base.h"
#include "decision_center.h"
#include "local_path_smoother.h"

namespace Planning
{
  /// \brief Generate a drivable local path from reference line + decision points.
  ///
  /// This component takes:
  /// - the reference line (global guidance),
  /// - coarse decision points from the decision module ("key frames" in Frenet),
  /// - ego vehicle state (current s, speed, acceleration),
  ///
  /// and produces a local path that is smooth and suitable for downstream control/tracking.
  ///
  /// \details
  /// The overall workflow in the current implementation is:
  /// 1) Sample a set of longitudinal stations ahead of ego (waypoints in Frenet s).
  /// 2) For each sampled s, find which two decision points it lies between and compute the lateral offset by a
  ///    quintic polynomial segment (smoothly connecting the two decision points).
  /// 3) Smooth the Frenet local path.
  /// 4) Convert Frenet points back to Cartesian using the reference line projection at the same s.
  /// 5) Post-compute the projected-point parameters (rs/rtheta/rkappa/rdkappa) for the generated local path.
  ///
  /// \note Preconditions / assumptions
  /// - \c referenceLine.refer_line is expected to have valid \c rs/rtheta/rkappa/rdkappa fields (typically computed by
  ///   Curve::calculateProjectedPointParameters() in the upstream pipeline).
  /// - Decision points are expected to be ordered by increasing \c s.
  class LocalPathPlanner
  {
  public:
    /// \brief Construct and initialize the local path planner.
    ///
    /// \details
    /// - Reads planner configuration via ConfigReader.
    /// - Creates the local path smoother.
    LocalPathPlanner();
    LocalPathPlanner(const LocalPathPlanner &) = delete;
    LocalPathPlanner operator=(const LocalPathPlanner &) = delete;
    ~LocalPathPlanner() = default;

    /// \brief Get the last generated local path for RViz visualization.
    ///
    /// \return nav_msgs::msg::Path: A copy of the cached RViz path.
    inline nav_msgs::msg::Path getLocalPathRviz() const { return localPathRviz; }

    /// \brief Get the last generated local path message.
    ///
    /// \return base_msgs::msg::LocalPath: A copy of the cached local path.
    inline base_msgs::msg::LocalPath getLocalPath() const { return localPath; }

    /// \brief Generate a local path based on reference line, decision points and ego state.
    ///
    /// \details
    /// The generated local path is built in Frenet first (s-l and derivatives), smoothed, and then converted to
    /// Cartesian (x-y, heading, curvature). The projection used in the Frenet-to-Cartesian conversion is obtained from
    /// the reference line by searching the point whose stored \c rs is closest to the queried \c s.
    ///
    /// @startuml
    /// start
    /// :initializeLocalPath();
    /// :wayPoint_s = ego.s;
    /// while (need more local path points?) is (yes)
    ///   :wayPoint_s += ego.ds_dt;
    ///   if (wayPoint_s beyond referenceLine end?) then (yes)
    ///     break
    ///   endif
    ///
    ///   :Create a LocalPathPoint at this s;
    ///   :Set ds_dt/dds_dt from ego;
    ///   :Init l/dl_ds/ddl_ds to 0;
    ///
    ///   :Find the decision-point segment that contains wayPoint_s;
    ///   if (found segment?) then (yes)
    ///     :Use a 5th-order polynomial to smoothly connect
    ///      segment start/end lateral offsets;
    ///     :Evaluate l, lateral slope, lateral curvature-trend at wayPoint_s;
    ///   endif
    ///
    ///   :Append point into localPath;
    /// endwhile (no)
    ///
    /// :Smooth localPath in Frenet (LocalPathSmoother);
    ///
    /// while (for each localPath point?) is (yes)
    ///   :Use point.s as query rs;
    ///   :matchIndex = findMatchPointIndex(referenceLine, rs);
    ///   :Build ProjectedPointInfo from referenceLine[matchIndex];
    ///   :FrenetToCartesian(point, projectedPoint) -> (x,y,theta,kappa);
    ///   :Fill pose/orientation and theta/kappa;
    /// endwhile (no)
    ///
    /// :calculateProjectedPointParameters(localPath);
    /// :return localPath;
    /// stop
    /// @enduml
    ///
    /// \param[in]  referenceLine  Reference line with precomputed projected parameters
    /// \param[in]  decision       DecisionCenter that provides ordered decision points in Frenet (s,l)
    /// \param[in]  egoCarInfo     Ego vehicle state provider (s, ds/dt, dds/dt)
    ///
    /// \return base_msgs::msg::LocalPath: Generated local path in Cartesian with projected parameters filled.
    base_msgs::msg::LocalPath generateLocalPath(const base_msgs::msg::Referline &referenceLine,
                                                const std::shared_ptr<DecisionCenter> &decision,
                                                const std::shared_ptr<VehicleInfoBase> &egoCarInfo);

    /// \brief Convert the cached local path into a nav_msgs::Path for RViz.
    ///
    /// \details
    /// This function uses the internally cached \c localPath and copies its poses into \c localPathRviz.
    ///
    /// \return nav_msgs::msg::Path: RViz visualization path.
    nav_msgs::msg::Path generateLocalPathRviz();

    /// \brief Initialize internal local path buffer before generating a new local path.
    ///
    /// \details
    /// - Sets the header frame id using the map config.
    /// - Sets the timestamp to current time.
    /// - Clears previously generated points.
    void initializeLocalPath();

  private:
    std::unique_ptr<ConfigReader> localPathConfigReader;  ///< Config reader for local path planner
    nav_msgs::msg::Path localPathRviz;                    ///< Cached local path for RViz visualization
    base_msgs::msg::LocalPath localPath;                  ///< Cached local path to be published
    std::shared_ptr<LocalPathSmoother> localPathSmoother; ///< Local path smoother
  };
} // namespace Planning
#endif // ! LOCAL_PATH_PLANNER_H_
