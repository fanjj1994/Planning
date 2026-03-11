#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning
{
  /// \brief  Normal global planner that generates a global path by averaging midline and right boundary of the PNC map.
  ///
  /// \details
  /// This planner implements the simplest global path strategy: for each point index, it computes the midpoint
  /// between the map's midline and right boundary, effectively producing a path along the right-half center of the
  /// lane. It reads configuration via ConfigReader on construction and stores the result in the inherited \c globalPath
  /// member.
  class GlobalPlannerNormal final : public GlobalPlannerBase
  {
  public:
    /// \brief  Construct a GlobalPlannerNormal instance.
    ///
    /// \details
    /// Initializes the configuration reader via ConfigReader::readGlobalPathConfig() and
    /// sets \c globalPlannerType to GlobalPlannerType::NORMAL.
    GlobalPlannerNormal();
    GlobalPlannerNormal(const GlobalPlannerNormal&) = delete;
    GlobalPlannerNormal& operator=(const GlobalPlannerNormal&) = delete;
    ~GlobalPlannerNormal() = default;

    /// \brief  Search and generate a global path from the given PNC map.
    ///
    /// \details
    /// For each point index \c i in \c pncMap.midline, the path point is computed as
    /// the average of the midline point and the right boundary point:
    ///   - \f$ x_i = \frac{midline_i.x + right\_boundary_i.x}{2} \f$
    ///   - \f$ y_i = \frac{midline_i.y + right\_boundary_i.y}{2} \f$
    ///
    /// The orientation of each pose is set to identity (all zeros).
    ///
    /// @startuml
    /// start
    /// :Set header (frame_id, stamp);
    /// :Clear existing globalPath.poses;
    /// :midLineSize = pncMap.midline.points.size();
    /// while (i < midLineSize?) is (yes)
    ///   :x = (midline[i].x + right_boundary[i].x) / 2.0;
    ///   :y = (midline[i].y + right_boundary[i].y) / 2.0;
    ///   :emplace_back to globalPath.poses;
    /// endwhile (done)
    /// :return globalPath;
    /// stop
    /// @enduml
    ///
    /// \param[in]  pncMap  PNC map containing midline and right_boundary with equal-sized point arrays
    ///
    /// \return     nav_msgs::msg::Path: Generated global path with header matching the input map
    nav_msgs::msg::Path searchGlobalPath(const base_msgs::msg::PNCMap&) override;

  private:
  };
} // namespace Planning
#endif // ! GLOBAL_PLANNER_NORMAL_H_
