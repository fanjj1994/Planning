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
  /// Each enumerator describes what the planning module should do at the
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

  /// \brief A waypoint expressed in Frenet (SL) coordinates that carries a behavioral decision.
  ///
  /// Decision points are produced by DecisionCenter::makePathDecision() and consumed by
  /// the local path planner to generate a lateral offset profile that avoids obstacles
  /// or brings the vehicle to a controlled stop.
  struct SLPoint
  {
    float64 s;           ///< Longitudinal position along the reference line [m].
    float64 l;           ///< Lateral offset relative to the reference line center-line
                         ///<  (positive = left, negative = right) [m].
    SLPointType type;    ///< Behavioral intent at this point; see SLPointType.
    float64 speed_limit; ///< Speed limit that shall be respected from this point onward [m/s].
  };

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
    /// #decisionPoints and can be retrieved via getDecisionPoints().
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
    ///       #decisionPoints. If no relevant obstacle is found, #decisionPoints will also
    ///       remain empty after the call.
    void makePathDecision(const std::shared_ptr<VehicleInfoBase> &egoCarInfo,
                          const std::vector<std::shared_ptr<VehicleInfoBase>> &tpInfoList);

    /// \brief Resets the decision state at the beginning of each planning cycle.
    ///
    /// Clears #decisionPoints so that stale decisions from the previous cycle do not
    /// pollute the current one. Must be called before re-running makePathDecision().
    void decisionInitialize();

    /// \brief Returns a read-only reference to the current decision point list.
    ///
    /// The returned reference is valid until the next call to makePathDecision() or
    /// decisionInitialize(). Downstream planners should copy the data if they need to
    /// retain it across planning cycles.
    ///
    /// \return Const reference to the internal vector of SLPoint decision waypoints.
    inline const std::vector<SLPoint> &getDecisionPoints() const { return decisionPoints; }

  private:
    /// \brief Config reader used to load all planning parameters at construction time.
    std::unique_ptr<ConfigReader> decisionConfigReader;

    /// \brief Ordered list of decision waypoints produced in the current planning cycle.
    std::vector<SLPoint> decisionPoints;

    ///< Minimum allowed value for #decisionMakingLeadPoint (clamping lower bound).
    static constexpr uint8 LEADPTMINNUM{ 30U };

    ///< Maximum allowed value for #decisionMakingLeadPoint (clamping upper bound).
    static constexpr uint8 LEADPTMAXNUM{ 40U };

    ///< Minimum decision-making longitudinal distance regardless of ego speed [m].
    static constexpr float32 DMMINLENGTH{ 30.0f };

    ///< Speed threshold below which a TP is treated as quasi-static [m/s].
    static constexpr float32 MINSPEED{ 0.03f };
  }; // class DecisionCenter
} // namespace Planning
#endif // ! DECISION_CENTER_H_
