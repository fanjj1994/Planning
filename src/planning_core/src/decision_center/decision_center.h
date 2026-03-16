#ifndef DECISION_CENTER_H_
#define DECISION_CENTER_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "ego_car_base.h"
#include "tp_base.h"
#include <vector>
#include <algorithm>

namespace Planning
{
  /// \brief Enumerates the behavioral decision types assigned to an SLPoint.
  ///
  /// Each enumerator describes what the PATH planning module should do at the
  /// corresponding longitudinal position along the reference line.
  enum class SLPointType : uint8
  {
    DECISION_UNKNOWN = 0U,        ///< Decision type has not been determined yet.
    DECISION_LEFT_OVERTAKE = 1U,  ///< Ego vehicle shall overtake the obstacle from the left side.
    DECISION_RIGHT_OVERTAKE = 2U, ///< Ego vehicle shall overtake the obstacle from the right side.
    DECISION_STOP = 3U,           ///< Ego vehicle shall decelerate to a stop at this point.
    DECISION_START = 4U,          ///< Marks the beginning of the decision-effect zone along the reference line.
    DECISION_END = 5U,            ///< Marks the end of the decision-effect zone; ego car may resume normal driving.
    DECISION_DEFAULT = 255U       ///< Sentinel / uninitialized value.
  };

  /// \brief Enumerates the behavioral decision types assigned to an STPoint.
  ///
  /// Each enumerator describes what the SPEED planning module should do at the
  /// corresponding longitudinal position along the local path.
  enum class STPointType : uint8
  {
    DECISION_UNKNOWN = 0U,         ///< Decision type has not been determined yet.
    DECISION_YIELD = 1U,           ///< Ego vehicle shall yield to the obstacle at this point.
    DECISION_ASSERTIVE_DRIVE = 2U, ///< Ego vehicle shall assertively drive through the collision zone.
    DECISION_STOP_OR_FOLLOW = 3U,  ///< Ego vehicle shall decelerate to a stop or follow the obstacle at this point.
    DECISION_START = 4U,           ///< Marks the beginning of the decision-effect zone in speed decisions.
    DECISION_END = 5U,             ///< Marks the end of the decision-effect zone in speed decisions.
    DECISION_DEFAULT = 255U        ///< Sentinel / uninitialized value.
  };

  /// \brief A waypoint expressed in Frenet (SL) coordinates that carries a behavioral decision.
  ///
  /// Decision points are produced by DecisionCenter::makePathDecision() and consumed by
  /// the local path planner to generate a lateral offset profile that avoids obstacles
  /// or brings the vehicle to a controlled stop.
  struct SLPoint
  {
    float64 s; ///< Longitudinal position along the reference line [m].
    float64 l; ///< Lateral offset relative to the reference line center-line(positive = left, negative = right) [m].
    SLPointType type;    ///< Behavioral intent at this point; see SLPointType.
    float64 speed_limit; ///< Speed limit that shall be respected from this point onward [m/s].
  };

  /// \brief A decision waypoint in the ST (longitudinal position – time) graph that carries a behavioral intent
  ///        for the speed planning module.
  ///
  /// STPoint instances are produced by DecisionCenter::makeSpeedDecision() and consumed by the local speed planner
  /// (QP optimizer). Together they form a piecewise-linear "guide curve" in the ST graph that encodes both geometric
  /// constraints (position and velocity at sampled times) and logical constraints (the driving intent expressed by
  /// #type). The QP optimizer uses this guide curve as a reference and boundary condition while generating a
  /// smooth, physically feasible speed profile.
  ///
  /// Coordinate convention:
  /// - The s-axis is the longitudinal displacement along the already-planned local path, with the origin at the
  ///   start of the speed-planning window (i.e. #s = 0 corresponds to \c decisionMakingLeastDistance ahead of the
  ///   ego vehicle).
  /// - The t-axis is absolute elapsed time [s] from the start of the current planning cycle.
  struct STPoint
  {
    float64 t;           ///< Time coordinate of this decision waypoint in the ST graph [s].
                         ///<   Together with #s_2path it defines one vertex of the piecewise-linear guide curve,
                         ///<   answering "at time \c t, the vehicle should have reached position \c s_2path".
    float64 s_2path;     ///< Longitudinal displacement along the local path at time #t, relative to the start of the
                         ///<   speed-planning window [m]. Positive values indicate a position ahead of the window
                         ///<   origin; see coordinate convention above.
    float64 ds_dt_2path; ///< First derivative of #s_2path with respect to #t, i.e. the reference velocity at this
                         ///<   decision point [m/s]. Provides the QP optimizer with a first-order target so that
                         ///<   the smoothed profile has a well-conditioned starting point. Typically set to the
                         ///<   tracked obstacle's longitudinal speed (follow-car scenario) or to the applicable
                         ///<   speed limit.
    float64 t0;          ///< Time anchor [s]: the elapsed time at which the traffic participant (TP) was first
                         ///<   incorporated into the current speed decision. Set via \c tpInfo->updateT0() and read
                         ///<   back with \c tpInfo->getT0(). Together with #s0 it pins the TP's ST trajectory line
                         ///<   to a known reference point, enabling the planner to compute relative displacements
                         ///<   and predict future conflict windows.
    float64 s0;          ///< Longitudinal position of the TP at time #t0, expressed in the speed-planning window
                         ///<   coordinate frame [m]. Computed as:
                         ///<   \code
                         ///<     s0 = tpSDistanceToEgoCar + ds_dt_tp * t0 - decisionMakingLeastDistance
                         ///<   \endcode
                         ///<   where \c tpSDistanceToEgoCar is the current true ego-to-TP gap, \c ds_dt_tp * t0
                         ///<   projects the TP forward by the elapsed tracking time, and
                         ///<   \c decisionMakingLeastDistance shifts the origin to the window start. The TP's full
                         ///<   ST trajectory is then \f$ s(t) = s_0 + \dot{s}_\text{tp}(t - t_0) \f$.
    float64 t_in;        ///< Entry time of the obstacle's ST-graph shadow [frames]: the earliest time at which the
                         ///<   TP occupies the path cross-section. For obstacles already on the path when the
                         ///<   decision is made (sub-branch 1a), \c t_in = 0. For laterally-approaching obstacles
                         ///<   (sub-branch 1b), \c t_in = tpToPathTime - timeToCrossHalfWidth.
    float64 t_out;       ///< Exit time of the obstacle's ST-graph shadow [frames]: the latest time at which the
                         ///<   TP occupies the path cross-section. For statically-blocking obstacles (1a),
                         ///<   \c t_out = speed_size_ (end of the planning window). For laterally-moving obstacles
                         ///<   (1b), \c t_out = tpToPathTime + timeToCrossHalfWidth.
    STPointType type;    ///< Semantic label that locks the QP optimizer to a specific homotopy class.
                         ///<   - \c DECISION_YIELD            : ego trajectory must pass *below* the obstacle's
                         ///<                                    ST shadow (decelerate and yield until \c t_out).
                         ///<   - \c DECISION_ASSERTIVE_DRIVE  : ego trajectory must pass *above* the shadow
                         ///<                                    (accelerate and clear before \c t_in).
                         ///<   - \c DECISION_STOP_OR_FOLLOW   : ego shall not exceed the TP's position by more
                         ///<                                    than \c safe_dis_lon; degenerates to a hard stop
                         ///<                                    when \c ds_dt_2path = 0 and to cruise-follow when
                         ///<                                    \c ds_dt_2path > 0.
                         ///<   - \c DECISION_START / \c DECISION_END : delimit the interval over which the
                         ///<                                    enclosing decision type applies.
    float64 speed_limit; ///< Speed limit that shall be respected from this point onward [m/s].
  };

  ///< Speed threshold below which a TP is treated as quasi-static [m/s].
  static constexpr float32 MINSPEED{ 0.03f };

  /// \brief Front-end decision module for the EM-Planner local planning pipeline.
  ///
  /// DecisionCenter converts raw traffic-participant (TP) information into a compact
  /// list of SLPoint waypoints that encode *what* the ego vehicle should do at each
  /// longitudinal position along the reference line (overtake left / right, or stop).
  /// The output is passed downstream to the local path and speed planners.
  ///
  /// Decision logic overview:
  /// 1. For every TP that is within the longitudinal planning horizon and inside the
  ///    road corridor, the module checks whether the TP is quasi-static (low speed,
  ///    no significant lateral movement).
  /// 2. For quasi-static TPs it evaluates, in priority order:
  ///    - Left overtake  : feasible if the gap between the TP left bounding-box edge
  ///      and the left road boundary is wider than the ego vehicle plus two lateral
  ///      safety margins.
  ///    - Right overtake : same check on the right side.
  ///    - Stop           : if neither side offers enough clearance the ego vehicle is
  ///      commanded to stop at a safe longitudinal distance in front of the TP.
  /// 3. A DECISION_START point is prepended and, when the last point is not a stop,
  ///    a DECISION_END point is appended so that downstream planners know the extent
  ///    of the decision-effect zone.
  ///
  /// \note Moving TPs (getDlDt() or getDsDt() above threshold) are currently not
  ///       handled; the corresponding branch is left as a TODO.
  class DecisionCenter
  {
  public:
    /// \brief Constructs a DecisionCenter and initializes all parameters from the config file.
    ///
    /// Reads the planning configuration via ConfigReader and derives:
    /// - Road boundary lateral distances (leftBoundaryDistance, rightBoundaryDistance).
    /// - Decision lead point count (decisionMakingLeadPoint).
    /// - Reference line end distance (referenceLineEndDistance).
    DecisionCenter();

    /// \brief Default destructor.
    ~DecisionCenter() = default;

    /// \brief Core entry point: generates the decision point list for one planning cycle.
    ///
    /// Iterates over all traffic participants in \p tpInfoList, filters irrelevant ones,
    /// and for each quasi-static obstacle inside the road corridor decides whether to
    /// overtake (left or right) or stop. The resulting SLPoint list is stored in
    /// #pathDecisionPoints and can be retrieved via getPathDecisionPoints().
    ///
    /// @startuml
    /// start
    /// if (Any traffic participants available?) then (no)
    ///   :Log and return - nothing to decide;
    ///   stop
    /// endif
    /// :Clear stale decision points from the previous cycle;
    /// :Scale the decision horizon based on ego speed and lead point count;
    /// while (For each traffic participant) is (next TP)
    ///   :Compute longitudinal distance from ego to TP;
    ///   if (Is TP beyond the reference line end\nor already passed the ego vehicle?) then (yes)
    ///     :Ignore TP - outside planning horizon;
    ///   elseif (Is TP inside the road corridor?) then (yes)
    ///     if (Is TP quasi-static?\nlow lateral speed and slower than half ego speed) then (yes)
    ///       :Predict the s-position where ego would\nencounter the TP using approach time;
    ///       :Compute TP lateral bounding box edges\n(left edge and right edge);
    ///       :Measure clearance between TP and each road boundary;
    ///       if (Is left overtake feasible?\nleft gap wider than ego width plus two safety margins) then (yes)
    ///         :Set target lateral offset to the center of\nthe left gap between TP and left road boundary;
    ///         :Emit DECISION_LEFT_OVERTAKE point;
    ///       elseif (Is right overtake feasible?\nright gap wider than ego width plus two safety margins) then (yes)
    ///         :Set target lateral offset to the center of\nthe right gap between TP and right road boundary;
    ///         :Emit DECISION_RIGHT_OVERTAKE point;
    ///       else (neither side feasible)
    ///         :Set stop position safe_dis_lon ahead of the TP\nto let the TP clear the path first;
    ///         :Emit DECISION_STOP point;
    ///         :Stop supersedes all further decisions - break loop;
    ///         break
    ///       endif
    ///     else (TP is moving)
    ///       :TODO - handle moving TP;
    ///     endif
    ///   else (outside road)
    ///     :Ignore TP - not in the drivable corridor;
    ///   endif
    /// endwhile (all TPs processed)
    /// if (Any decision points generated?) then (no)
    ///   :Log and return - free-drive, no action needed;
    ///   stop
    /// endif
    /// :Prepend DECISION_START point\nto give the planner a smooth lead-in ramp;
    /// if (Last decision is not a stop?) then (yes)
    ///   :Append DECISION_END point\nto signal where ego may return to center line;
    /// endif
    /// stop
    /// @enduml
    ///
    /// \param[in]  egoCarInfo   Shared pointer to the ego vehicle's current state (pose, speed,
    ///                          dimensions) expressed in Frenet coordinates.
    /// \param[in]  tpInfoList   List of traffic participants, each providing Frenet-frame state
    ///                          and physical dimensions. The list must remain valid for the
    ///                          duration of the call.
    ///
    /// \note If \p tpInfoList is empty, the function returns immediately without modifying
    ///       #pathDecisionPoints. If no relevant obstacle is found, #pathDecisionPoints will also
    ///       remain empty after the call.
    void makePathDecision(const std::shared_ptr<VehicleInfoBase> &egoCarInfo,
                          const std::vector<std::shared_ptr<VehicleInfoBase>> &tpInfoList);

    /// \brief Core entry point for speed decisions: generates the STPoint list for one planning cycle.
    ///
    /// Iterates over all traffic participants in \p tpInfoList, applies a longitudinal range filter,
    /// and for each TP that laterally overlaps the local path decides between three strategies:
    ///
    /// - **STOP_OR_FOLLOW** (sub-branch 1a): TP is laterally static and slower than ego. Ego must
    ///   maintain \c safe_dis_lon behind the TP for the entire planning window. Degenerates to a
    ///   hard stop when the TP's longitudinal speed is near zero.
    /// - **YIELD** (sub-branch 1b): TP is laterally crossing the path and ego would arrive during
    ///   the conflict window [t_in, t_out]. Ego decelerates to let the TP clear before proceeding.
    /// - **ASSERTIVE_DRIVE** (sub-branch 1b): TP is laterally crossing but ego would arrive just
    ///   before \c t_in (within the longitudinal safety buffer). Ego maintains/increases speed to
    ///   clear the conflict zone ahead of the TP.
    ///
    /// Only the first qualifying TP generates a decision; the loop breaks immediately after emitting
    /// an STPoint. After the loop, a DECISION_START framing point is always prepended. A
    /// DECISION_END framing point is appended for YIELD and ASSERTIVE_DRIVE decisions only
    /// (analogous to how DECISION_STOP in path planning omits an end marker).
    ///
    /// @startuml
    /// start
    /// if (Any traffic participants available?) then (no)
    ///   :Log and return - nothing to decide;
    ///   stop
    /// endif
    /// :Clear stale speed decision points from the previous cycle;
    /// :Compute decision horizon distance from configured set speed and lead frame count;
    /// :Compute SimpleTTB as mid-point of the speed planning window;
    /// while (For each traffic participant) is (next TP)
    ///   :Compute corrected longitudinal distance from ego to TP\nalong the local path;
    ///   if (Is TP beyond decision horizon\nor already passed ego with safe margin?) then (yes)
    ///     :Ignore TP - outside longitudinal range;
    ///   elseif (Does TP laterally overlap the path cross-section?) then (yes)
    ///     if (Is TP laterally static?\ndl/dt below threshold) then (yes)
    ///       if (Is TP faster than ego plus speed margin?) then (yes)
    ///         :Ignore TP - pulling away, no collision risk;
    ///       else (TP is slower or stationary)
    ///         :Record TP tracking time anchor and ST reference line origin;
    ///         :Set conflict window from now to end of planning window\n(TP blocks entire horizon);
    ///         :Place ST decision vertex at mid-window time\nat safe following distance behind TP;
    ///         :Set reference speed to TP longitudinal speed;
    ///         :Emit DECISION_STOP_OR_FOLLOW point;
    ///         :STOP_OR_FOLLOW supersedes all further decisions - break loop;
    ///         break
    ///       endif
    ///     else (TP is laterally moving)
    ///       if (Ego configured set speed is near zero?) then (yes)
    ///         :Ignore TP - time calculation undefined at zero speed;
    ///       else
    ///         :Estimate time for ego to reach TP longitudinal position at set speed;
    ///         :Compute time for TP lateral centre to reach path centre-line;
    ///         if (Is TP moving away from the path?) then (yes)
    ///           :Ignore TP - diverging, no future conflict;
    ///         else (TP is approaching the path)
    ///           :Record TP tracking time anchor and ST reference line origin;
    ///           :Compute conflict window start and end\nusing TP half-width and longitudinal safety margin;
    ///           if (Ego would arrive during or just after the conflict window?) then (yes)
    ///             :Place ST vertex at conflict window exit time\nat safe distance behind TP;
    ///             :Set reference speed to configured set speed;
    ///             :Emit DECISION_YIELD point;
    ///             :YIELD supersedes all further decisions - break loop;
    ///             break
    ///           elseif (Ego would arrive just before the conflict window?) then (yes)
    ///             :Place ST vertex at conflict window entry time\nat safe distance ahead of TP;
    ///             :Set reference speed to configured set speed;
    ///             :Emit DECISION_ASSERTIVE_DRIVE point;
    ///             :ASSERTIVE_DRIVE supersedes all further decisions - break loop;
    ///             break
    ///           else (Ego arrival is safely outside conflict window)
    ///             :Ignore TP - no collision risk;
    ///           endif
    ///         endif
    ///       endif
    ///     endif
    ///   else (TP does not overlap path)
    ///     :Ignore TP - not in path cross-section;
    ///   endif
    /// endwhile (all TPs processed)
    /// if (Any speed decision points generated?) then (no)
    ///   :Log and return - free drive, no action needed;
    ///   stop
    /// endif
    /// :Prepend DECISION_START point at TP tracking origin\nto give the QP a smooth ramp-in condition;
    /// if (Decision type is not STOP_OR_FOLLOW?) then (yes)
    ///   :Append DECISION_END point at end of planning window\nextrapolated at configured set speed;
    /// endif
    /// stop
    /// @enduml
    ///
    /// \param[in]  egoCarInfo   Shared pointer to the ego vehicle's current state (speed, dimensions).
    /// \param[in]  tpInfoList   List of traffic participants with Frenet-frame state projected onto
    ///                          the local path. The list must remain valid for the duration of the call.
    ///
    /// \note If \p tpInfoList is empty, or no TP qualifies, #speedDecisionPoints remains empty.
    void makeSpeedDecision(const std::shared_ptr<VehicleInfoBase> &egoCarInfo,
                           const std::vector<std::shared_ptr<VehicleInfoBase>> &tpInfoList);

    /// \brief Resets the path decision state at the beginning of each planning cycle.
    ///
    /// Clears #pathDecisionPoints so that stale decisions from the previous cycle do not
    /// pollute the current one. Must be called before re-running makePathDecision().
    void pathDecisionInitialize();

    /// \brief Resets the speed decision state at the beginning of each planning cycle.
    ///
    /// Clears #speedDecisionPoints so that stale decisions from the previous cycle do not
    /// pollute the current one. Must be called before re-running makeSpeedDecision().
    void speedDecisionInitialize();

    /// \brief Returns a read-only reference to the current path decision point list.
    ///
    /// The returned reference is valid until the next call to makePathDecision() or
    /// pathDecisionInitialize(). Downstream planners should copy the data if they need to
    /// retain it across planning cycles.
    ///
    /// \return Const reference to the internal vector of SLPoint decision waypoints.
    inline const std::vector<SLPoint> &getPathDecisionPoints() const { return pathDecisionPoints; }

    inline const std::vector<STPoint> &getSpeedDecisionPoints() const { return speedDecisionPoints; }

  private:
    /// \brief Config reader used to load all planning parameters at construction time.
    std::unique_ptr<ConfigReader> decisionConfigReader;

    /// \brief Ordered list of path decision waypoints produced in the current planning cycle.
    std::vector<SLPoint> pathDecisionPoints;

    /// \brief Ordered list of speed decision waypoints produced in the current planning cycle.
    std::vector<STPoint> speedDecisionPoints;

    ///< Minimum decision-making longitudinal distance regardless of ego speed [m].
    static constexpr float32 DMMINLENGTH{ 30.0f };

    ///< Speed safety margin added to the ego vehicle's speed when evaluating if Tp is faster than ego [m/s].
    static constexpr float32 SPEEDMARGIN{ 0.5f };

  }; // class DecisionCenter
} // namespace Planning
#endif // ! DECISION_CENTER_H_
