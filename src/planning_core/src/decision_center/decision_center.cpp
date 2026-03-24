/**
 * \file decision_center.cpp
 * \brief Implementation of the DecisionCenter class.
 *
 * Translates per-cycle traffic-participant states into a compact ordered list of
 * SLPoint decision waypoints. Each waypoint encodes a behavioral intent (left overtake,
 * right overtake, or stop) at a specific longitudinal position along the Frenet reference
 * line, together with framing DECISION_START / DECISION_END waypoints consumed by the
 * downstream local path planner.
 */

#include "decision_center.h"

namespace Planning
{
  DecisionCenter::DecisionCenter() : pathDecisionPoints()
  {
    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "[Constructor] DecisionCenter is running");

    // Read config file for decision center component
    decisionConfigReader = std::make_unique<ConfigReader>();
    decisionConfigReader->readDecisionConfig();
  }

  void DecisionCenter::makePathDecision(const std::shared_ptr<VehicleInfoBase>& egoCarInfo,
                                        const std::vector<std::shared_ptr<VehicleInfoBase>>& tpInfoList)
  {
    if (tpInfoList.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"), "[Constructor] No traffic participant information available");
      return;
    }

    // Clear stale decision data from the previous planning cycle before re-computing.
    pathDecisionInitialize();

    // Minimum longitudinal separation required for the ego vehicle to make/prepare a decision with respect to a
    // traffic participant (TP) [m]. In this implementation it acts as a per-cycle decision horizon/threshold:
    // - TPs further than this distance behind the ego are ignored (already passed).
    // - Used as the lead-in/lead-out buffer when placing DECISION_START / DECISION_END waypoints.
    // Computed each cycle as max(ego_dsDt * decisionMakingLeadPoint, DMMINLENGTH) so it scales with ego speed while
    // respecting a minimum decision horizon.
    float64 decisionMakingLeastDistance = 0.0;

    // Lateral coordinate from the ego vehicle center to the left road boundary in Frenet coordinates (positive value)
    // [m]. Derived at construction as 1.5 x lane_width, assuming the ego vehicle travels in the center of the
    // right lane on a two-lane road.
    const float64 leftBoundaryDistance = static_cast<float64>(decisionConfigReader->getPNCMap().lane_width_ * 1.5f);

    // Lateral coordinate of the right road boundary in Frenet frame (negative value, since right of the reference
    // line is the negative-l direction) [m]. Derived as -(0.5 x lane_width), assuming the ego vehicle travels
    // in the center of the right lane on a two-lane road.
    const float64 rightBoundaryDistance = -(static_cast<float64>(decisionConfigReader->getPNCMap().lane_width_ * 0.5f));

    // Number of path points that constitute the "lead" zone ahead of the ego vehicle [-].
    // Clamped to [30U, 40U]. Used to scale #decisionMakingLeastDistance with respect to the ego speed
    // so that the decision horizon is proportional to the configured local path length.
    const uint32 decisionMakingLeadPoint =
        std::max(std::min(decisionConfigReader->getLocalPath().path_size_ - 50U, 40U), 30U);

    // Longitudinal extent of the reference line in front of the ego vehicle, computed as reference_line_front_size x
    // segment_len [m]. TPs with a longitudinal distance exceeding this value are outside the planned reference line and
    // are therefore excluded from decision making.
    const float64 referenceLineEndDistance = static_cast<float64>(decisionConfigReader->getReferenceLine().front_size_ *
                                                                  decisionConfigReader->getPNCMap().segment_len_);

    /// Temporary SLPoint built for each TP inside the loop; pushed into pathDecisionPoints when valid.
    SLPoint p{};

    /* Update decisionMakingLeastDistance for this cycle.
       It is computed as ego longitudinal speed (dS/dt) multiplied by the lead point count,
       which approximates the distance the ego vehicle travels during its reaction window.
       A minimum of DMMINLENGTH is enforced so that low-speed scenarios retain a safe planning horizon. */
    decisionMakingLeastDistance = static_cast<float64>(
        std::max(egoCarInfo->getDsDt() * decisionMakingLeadPoint, static_cast<float64>(DMMINLENGTH)));

    // Core: compute decision points based on the traffic participant information and the ego car information
    RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                "[Path Decision] Decision params: leftBound = %.2f, rightBound = %.2f, refLineEnd=%.2f, "
                "dmLeastDist=%.2f, tpCount=%zu",
                leftBoundaryDistance, rightBoundaryDistance, referenceLineEndDistance, decisionMakingLeastDistance,
                tpInfoList.size());
    RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                "[Path Decision] Ego car: s=%.2f, l=%.2f, ds_dt=%.2f, dl_dt=%.2f", egoCarInfo->getS(),
                egoCarInfo->getL(), egoCarInfo->getDsDt(), egoCarInfo->getDlDt());
    for (const auto& tpInfo : tpInfoList)
    {
      /* Longitudinal separation between the TP and the ego vehicle in Frenet coordinates.
         Positive value means the TP is ahead of the ego vehicle. */
      const float64 distanceToEgoCar = tpInfo->getS() - egoCarInfo->getS();
      RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                  "[Path Decision] TP[%d]: s=%.2f, l=%.2f, ds_dt=%.2f, dl_dt=%.2f, distToEgo=%.2f",
                  tpInfo->getVehicleID(), tpInfo->getS(), tpInfo->getL(), tpInfo->getDsDt(), tpInfo->getDlDt(),
                  distanceToEgoCar);

      /* Longitudinal range filter: skip TPs that are
         - beyond the reference line end (too far ahead to act on), or
         - more than decisionMakingLeastDistance behind the ego vehicle (already passed). */
      if ((distanceToEgoCar > referenceLineEndDistance) || (distanceToEgoCar < -decisionMakingLeastDistance))
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"), "[Path Decision] -> filtered: longitudinal range");
        continue;
      }
      // Tp is inside the corridor boundary or called road
      else if ((tpInfo->getL() < leftBoundaryDistance) && (tpInfo->getL() > rightBoundaryDistance))
      {
        /* Longitudinal speed threshold: a TP is treated as quasi-static when its forward
           speed is less than half the ego speed. This avoids false decisions against TPs
           that are moving nearly as fast as the ego vehicle. */
        const float64 tpLongitudinalSpeedThreshold = egoCarInfo->getDsDt() / 2.0f;

        // consider tp which is nearly static: no lateral movement and low longitudinal speed, which may cause collision
        // if ego car is close to it
        if ((std::fabs(tpInfo->getDlDt()) < MINSPEED) && (tpInfo->getDsDt() < tpLongitudinalSpeedThreshold))
        {
          /* Predict the longitudinal position (s) where the ego vehicle would reach the TP.
             Uses the relative closing speed to compute approach time, then projects the TP's
             future s position. Division by zero is guarded: if relative speed is zero the
             vehicles will never meet and approachTime defaults to 0. */
          const float64 relativeLongSpeed = egoCarInfo->getDsDt() - tpInfo->getDsDt();
          const float64 approachTime = (relativeLongSpeed != 0.0) ? (distanceToEgoCar / relativeLongSpeed) : 0.0;
          /// Predicted s-coordinate of the encounter point between ego and TP.
          p.s = tpInfo->getS() + tpInfo->getDsDt() * approachTime;

          /* Compute the TP's axis-aligned bounding box edges in the lateral (l) axis.
             Left boundary is positive (further left), right boundary is negative or smaller positive. */
          const float64 tpHalfWidth = tpInfo->getVehicleWidth() / 2.0f;
          const float64 tpBoundingBoxLeftBoundary = tpInfo->getL() + tpHalfWidth;
          const float64 tpBoundingBoxRightBoundary = tpInfo->getL() - tpHalfWidth;
          /// Gap between the TP's left bounding-box edge and the left road boundary.
          const float64 tpDistanceToLeftBoundary = leftBoundaryDistance - tpBoundingBoxLeftBoundary;
          /// Gap between the TP's right bounding-box edge and the right road boundary.
          const float64 tpDistanceToRightBoundary = tpBoundingBoxRightBoundary - rightBoundaryDistance;

          /* ============================================================================================================
             Decision core principle: if left side can be overtaken, then prioritize left overtaking; if not, then check
             right side; if neither side can be overtaken, then stop at the current position
             ============================================================================================================*/
          /* Left overtake feasibility: the gap to the left boundary must accommodate the ego vehicle width
             plus one safety margin on each side (2 × lat_safe_margin). */
          if (tpDistanceToLeftBoundary >
              egoCarInfo->getVehicleWidth() + decisionConfigReader->getDecision().lat_safe_margin_ * 2.0)
          {
            /* Target lateral position is the midpoint between the left road boundary and the TP's
               left bounding-box edge, placing the ego vehicle in the center of the available gap. */
            p.l = (leftBoundaryDistance + tpBoundingBoxLeftBoundary) / 2.0;
            p.type = SLPointType::DECISION_LEFT_OVERTAKE;
            pathDecisionPoints.emplace_back(p);
          }
          /* Right overtake feasibility: same clearance check on the right side. Only evaluated when
             left overtake is not feasible (strict priority: left > right > stop). */
          else if (tpDistanceToRightBoundary >
                   egoCarInfo->getVehicleWidth() + decisionConfigReader->getDecision().lat_safe_margin_ * 2.0)
          {
            /* Target lateral position is the midpoint between the right road boundary and the TP's
               right bounding-box edge. */
            p.l = (rightBoundaryDistance + tpBoundingBoxRightBoundary) / 2.0;
            p.type = SLPointType::DECISION_RIGHT_OVERTAKE;
            pathDecisionPoints.emplace_back(p);
          }
          /* Stop decision: neither side provides sufficient clearance. The ego vehicle is commanded
             to stop long_safe_margin metres ahead of the predicted encounter point and wait for the TP
             to clear the path. Processing subsequent TPs is aborted because a stop supersedes all
             further decisions. */
          else
          {
            p.l = 0.0f;
            p.s = p.s - decisionConfigReader->getDecision().long_safe_margin_;
            p.type = SLPointType::DECISION_STOP;
            RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                        "[Path Decision] Traffic participant ID %d cannot be overtaken and is blocking the path. "
                        "Decide to stop at s = %.2f m, l = %.2f m.",
                        tpInfo->getVehicleID(), p.s, p.l);
            pathDecisionPoints.emplace_back(p);
            // break to avoid processing following tps.
            break;
          }
        }
        // tp is active: moving at relatively high longitudinal speed or with lateral movement, which may not be handled
        // by current decision logic, so log and skip it.
        else
        {
          RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                      "[Path Decision] -> filtered: TP is moving at relatively high longitudinal speed or with lateral "
                      "speed(dl/dt=%.3f, ds/dt=%.2f)",
                      tpInfo->getDlDt(), tpInfo->getDsDt());
          // Todo: add this kind of active tp decision logic.
        }
      }
      // Tp is not inside the corridor boundary or called road
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                    "[Path Decision] -> filtered: outside corridor (l=%.2f, need %.2f < l < %.2f)", tpInfo->getL(),
                    rightBoundaryDistance, leftBoundaryDistance);
        // do nothing.
      }
    }

    if (pathDecisionPoints.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"), "[Path Decision] No decision point generated in this cycle");
      return;
    }

    /* Prepend a DECISION_START waypoint located decisionMakingLeastDistance before the first
       decision point. This gives the local planner a smooth lead-in ramp before the first lateral
       or longitudinal maneuver takes effect. */
    SLPoint decisionStartPoint{};
    decisionStartPoint.s = pathDecisionPoints.front().s - decisionMakingLeastDistance;
    decisionStartPoint.l = 0.0;
    decisionStartPoint.type = SLPointType::DECISION_START;
    pathDecisionPoints.emplace(pathDecisionPoints.begin(), decisionStartPoint);

    /* Append a DECISION_END waypoint only when the last action is NOT a stop.
       For a stop decision the vehicle does not continue past the stop point, so no
       end marker is needed. For overtake decisions the end point signals where the ego
       vehicle may return to the reference line center (l = 0). */
    if (pathDecisionPoints.back().type != SLPointType::DECISION_STOP)
    {
      SLPoint decisionEndPoint{};
      decisionEndPoint.s = pathDecisionPoints.back().s + decisionMakingLeastDistance;
      decisionEndPoint.l = 0.0;
      decisionEndPoint.type = SLPointType::DECISION_END;
      pathDecisionPoints.emplace_back(decisionEndPoint);
    }
    else
    {
      // do nothing.
    }
  }

  void DecisionCenter::makeSpeedDecision(const std::shared_ptr<VehicleInfoBase>& egoCarInfo,
                                         const std::vector<std::shared_ptr<VehicleInfoBase>>& tpInfoList)
  {
    if (tpInfoList.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                  "[Speed Decision] No traffic participant information available");
      return;
    }

    // Clear stale decision data from the previous planning cycle before re-computing.
    speedDecisionInitialize();

    /* Number of speed-profile frames that constitute the "lead" buffer at the front of the planning window [-].
       Clamped to [40, 50]. Used to derive the decision horizon distance and the SimpleTTB anchor below. */
    const float64 decisionMakingLeadFrame =
        static_cast<float64>(std::max(std::min(decisionConfigReader->getLocalSpeeds().speeds_size_ - 50U, 50U), 40U));

    /* Decision horizon [m]: farthest longitudinal distance at which a TP can still influence the speed plan.
       Uses the configured set speed (rather than real-time ego speed) for a stable, cycle-invariant horizon. */
    const float64 decisionMakingLeastDistance = decisionMakingLeadFrame * decisionConfigReader->getEgoCar().set_speed_;

    /* Simple Time-To-Boundary [frames]: representative mid-window time used to place the ST decision vertex.
       Taken as the average of (speed_size_ + lead_frames), balancing near-term accuracy with look-ahead range. */
    const float64 SimpleTTB =
        (static_cast<float64>(decisionConfigReader->getLocalSpeeds().speeds_size_) + decisionMakingLeadFrame) / 2.0;

    STPoint p{};

    for (const auto& tpInfo : tpInfoList)
    {
      /* True longitudinal distance from the ego vehicle's current position to the TP [m].
         getS2Path() returns the TP's projected s-coordinate relative to the local path's start point.
         However, the local path is generated starting one planning cycle ahead of the ego vehicle
         (path_start_s = ego_s + ds_dt, see LocalPathPlanner::generateLocalPath), so getS2Path()
         under-estimates the real ego-to-TP gap by exactly one frame's worth of longitudinal travel.
         Adding egoCarInfo->getDsDt() compensates for this offset and recovers the true separation:
           tpSDistanceToEgoCar = (s_TP_on_path - path_start_s) + ds_dt
                              = s_TP_on_path - ego_s */
      const float64 tpSDistanceToEgoCar = tpInfo->getS2Path() + egoCarInfo->getVehicleVelocity();

      /* Longitudinal range filter: skip TPs that are beyond the decision horizon or already well behind ego.
         The rear margin equals long_safe_margin to suppress oscillating yield/stop decisions near the ego bumper. */
      if ((tpSDistanceToEgoCar > decisionMakingLeastDistance) ||
          (tpSDistanceToEgoCar < -(decisionConfigReader->getDecision().long_safe_margin_)))
      {
        RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                    "[Speed Decision] -> filtered: longitudinal range (tpDistToEgo=%.2f, leastDist=%.2f)",
                    tpSDistanceToEgoCar, decisionMakingLeastDistance);
        continue;
      }

      float64 t_in{ 0.0 };
      float64 t_out{ 0.0 };

      /* ─── Branch 1: TP currently overlaps the path cross-section laterally ─── */
      if (std::fabs(tpInfo->getL2Path()) < (tpInfo->getVehicleWidth() / 2.0))
      {
        /* Sub-branch 1a: TP is laterally static (dl/dt ≈ 0) — parked or driving straight on the path. */
        if (std::fabs(tpInfo->getDlDt2Path()) < MINSPEED)
        {
          if (tpInfo->getDsDt2Path() > egoCarInfo->getDsDt() + SPEEDMARGIN)
          {
            // TP is pulling away faster than ego with a speed margin. No collision risk; skip this TP.
            continue;
          }
          else
          {
            /* TP is slower than ego (or stationary) and cannot be laterally overtaken (path decision's verdict).
               Decision: STOP_OR_FOLLOW — ego must not exceed the TP's longitudinal speed for the entire
               planning window. Degenerates to a hard stop when ds_dt_tp ≈ 0 (parked obstacle). */

            // Anchor the TP's ST trajectory line at the current tracking instant.
            tpInfo->updateT0();
            p.t0 = tpInfo->getT0();
            p.s0 = tpSDistanceToEgoCar + tpInfo->getDsDt2Path() * p.t0 - decisionMakingLeastDistance;

            // The TP blocks the path from now to the end of the planning window (no lateral escape possible).
            t_in = 0.0;
            t_out = static_cast<float64>(decisionConfigReader->getLocalSpeeds().speeds_size_);

            /* Project the safe-following position to the representative mid-window time (t0 + SimpleTTB):
               s_2path = current_distance - long_safe_margin + tp_speed × p.t
               This defines the ST vertex the QP must stay below. */
            p.t = p.t0 + SimpleTTB;
            p.s_2path = tpSDistanceToEgoCar - decisionConfigReader->getDecision().long_safe_margin_ +
                        tpInfo->getDsDt2Path() * p.t;
            p.ds_dt_2path = tpInfo->getDsDt2Path();
            p.type = STPointType::DECISION_STOP_OR_FOLLOW;
            speedDecisionPoints.emplace_back(p);
            RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                        "[Speed Decision] TP[%d] occupies path, slower than ego or unpassable. "
                        "STOP_OR_FOLLOW: t=%.2f s=%.2f ds/dt=%.2f (t_in=%.2f t_out=%.2f)",
                        tpInfo->getVehicleID(), p.t, p.s_2path, p.ds_dt_2path, t_in, t_out);
            tpInfo->setTInTOut(p.t, t_in, t_out);
            break; // STOP_OR_FOLLOW supersedes all subsequent TPs.
          }
        }
        /* Sub-branch 1b: TP is laterally moving across the path (dl/dt ≠ 0).
           The obstacle occupies the path only during a finite window [t_in, t_out]; ego must either
           yield (let it clear) or drive through assertively (pass before it arrives). */
        else
        {
        }
      }
      /* ─── Branch 2: TP currently does not overlap the path cross-section laterally ─── */
      else
      {
        // Skip if configured set speed is near zero — time-based calculations below would be undefined.
        if (std::fabs(decisionConfigReader->getEgoCar().set_speed_) < MINSPEED)
        {
          continue;
        }

        /* Estimated time for ego (at set_speed_) to reach the TP's current longitudinal position [frames].
           Used to determine whether ego arrives before or after the TP clears the path. */
        const float64 egoCarToTpSTimeUsingSetSpeed = tpSDistanceToEgoCar / decisionConfigReader->getEgoCar().set_speed_;

        /* Time for the TP's lateral centre to reach the path centre-line (l = 0) [frames].
           A negative value means the TP is already moving away from the path — no future conflict. */
        const float64 tpToPathTime = (0.0 - tpInfo->getL2Path()) / tpInfo->getDlDt2Path();

        if (tpToPathTime < 0.0)
        {
          // TP is diverging from the path. No collision risk; skip this TP.
          continue;
        }

        // Anchor the TP's ST trajectory at the current tracking instant.
        tpInfo->updateT0();
        p.t0 = tpInfo->getT0();
        p.s0 = tpSDistanceToEgoCar - decisionMakingLeastDistance;

        /* Build the ST-shadow conflict window [t_in, t_out] with bounding-box safety margins:
           - timeToCrossHalfWidthOfTp: time for the TP to travel one vehicle half-width laterally.
           - deltaT: longitudinal safe distance converted to a time margin using set_speed_. */
        const float64 deltaT =
            decisionConfigReader->getDecision().long_safe_margin_ / decisionConfigReader->getEgoCar().set_speed_;
        const float64 timeToCrossHalfWidthOfTp = (tpInfo->getVehicleWidth() / 2.0) / std::fabs(tpInfo->getDlDt2Path());
        t_in = tpToPathTime - timeToCrossHalfWidthOfTp;
        t_out = tpToPathTime + timeToCrossHalfWidthOfTp;
        RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                    "[Speed Decision] TP[%d] laterally crossing path: tpToPathTime=%.2f t_in=%.2f t_out=%.2f "
                    "egoArrivalTime=%.2f",
                    tpInfo->getVehicleID(), tpToPathTime, t_in, t_out, egoCarToTpSTimeUsingSetSpeed);

        if ((egoCarToTpSTimeUsingSetSpeed > tpToPathTime) && (egoCarToTpSTimeUsingSetSpeed < (t_out + deltaT)))
        {
          /* YIELD: ego would arrive while the TP is still crossing (or just after it barely clears).
             Ego decelerates to wait for the TP to fully exit the path.
             ST vertex at (t_out, safe_distance_behind_tp) — the earliest time ego may safely proceed. */
          p.t = t_out;
          p.s_2path = tpSDistanceToEgoCar - decisionConfigReader->getDecision().long_safe_margin_;
          p.ds_dt_2path = decisionConfigReader->getEgoCar().set_speed_;
          p.type = STPointType::DECISION_YIELD;
          speedDecisionPoints.emplace_back(p);
          RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                      "[Speed Decision] -> YIELD to TP[%d]: wait until t=%.2f, hold at s=%.2f", tpInfo->getVehicleID(),
                      p.t, p.s_2path);
        }
        else if ((egoCarToTpSTimeUsingSetSpeed < tpToPathTime) && (egoCarToTpSTimeUsingSetSpeed > (t_in - deltaT)))
        {
          /* ASSERTIVE DRIVE: ego would arrive just before the TP enters the path (within safety buffer).
             Ego maintains or increases speed to clear the TP's future conflict zone ahead of it.
             ST vertex at (t_in, safe_distance_ahead_of_tp) — the latest time ego must have passed. */
          p.t = t_in;
          p.s_2path = tpSDistanceToEgoCar + decisionConfigReader->getDecision().long_safe_margin_;
          p.ds_dt_2path = decisionConfigReader->getEgoCar().set_speed_;
          p.type = STPointType::DECISION_ASSERTIVE_DRIVE;
          speedDecisionPoints.emplace_back(p);
          RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                      "[Speed Decision] -> ASSERTIVE DRIVE past TP[%d]: clear before t=%.2f, target s=%.2f",
                      tpInfo->getVehicleID(), p.t, p.s_2path);
        }
        tpInfo->setTInTOut(p.t, t_in, t_out);
      }
    }

    /* ── Post-loop: add DECISION_START and (conditionally) DECISION_END framing points ── */
    if (speedDecisionPoints.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                  "[Speed Decision] No speed decision point generated in this cycle");
      return;
    }

    /* Save the decision type before modifying the vector (insert at front shifts the existing element). */
    const STPointType decisionType = speedDecisionPoints.front().type;

    /* Prepend DECISION_START: marks where the constrained speed zone begins in the ST graph.
       Placed at the TP's tracking origin (t0, s0) to provide the QP with a smooth ramp-in condition. */
    STPoint pStart{};
    pStart.t = speedDecisionPoints.front().t0;
    pStart.s_2path = speedDecisionPoints.front().s0;
    pStart.ds_dt_2path = decisionConfigReader->getEgoCar().set_speed_;
    pStart.type = STPointType::DECISION_START;
    speedDecisionPoints.emplace(speedDecisionPoints.begin(), pStart);

    /* Append DECISION_END: marks where the constrained speed zone ends in the ST graph.
       Placed at the end of the local speed profile to provide the QP with a smooth ramp-out condition. */
    STPoint pEnd{};
    pEnd.t = static_cast<float64>(decisionConfigReader->getLocalSpeeds().speeds_size_);
    pEnd.s_2path = speedDecisionPoints.back().s_2path +
                   speedDecisionPoints.back().ds_dt_2path * (pEnd.t - speedDecisionPoints.back().t);
    pEnd.ds_dt_2path = speedDecisionPoints.back().ds_dt_2path;
    pEnd.type = STPointType::DECISION_END;
    speedDecisionPoints.emplace_back(pEnd);

    RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                "[Speed Decision] Speed decision finalized: type=%d, total_points=%zu", static_cast<int>(decisionType),
                speedDecisionPoints.size());
  }

  void DecisionCenter::pathDecisionInitialize()
  {
    // Clear all path decision points produced in the previous cycle to prevent stale data from
    // propagating into the current planning iteration.
    pathDecisionPoints.clear();
  }

  void DecisionCenter::speedDecisionInitialize()
  {
    // Clear all speed decision points produced in the previous cycle to prevent stale data from
    // propagating into the current planning iteration.
    speedDecisionPoints.clear();
  }

} // namespace Planning
