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
  DecisionCenter::DecisionCenter() : decisionPoints()
  {
    RCLCPP_INFO(rclcpp::get_logger("decision_center"), "DecisionCenter is running");

    // Read config file for decision center component
    decisionConfigReader = std::make_unique<ConfigReader>();
    decisionConfigReader->readDecisionConfig();
  }

  void DecisionCenter::makePathDecision(const std::shared_ptr<VehicleInfoBase>& egoCarInfo,
                                        const std::vector<std::shared_ptr<VehicleInfoBase>>& tpInfoList)
  {
    if (tpInfoList.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"), "No traffic participant information available");
      return;
    }

    // Clear stale decision data from the previous planning cycle before re-computing.
    decisionInitialize();

    // Minimum longitudinal separation required for the ego vehicle to make/prepare a decision with respect to a
    // traffic participant (TP) [m]. In this implementation it acts as a per-cycle decision horizon/threshold:
    // - TPs further than this distance behind the ego are ignored (already passed).
    // - Used as the lead-in/lead-out buffer when placing DECISION_START / DECISION_END waypoints.
    // Computed each cycle as max(ego_dsDt * decisionMakingLeadPoint, DMMINLENGTH) so it scales with ego speed while
    // respecting a minimum decision horizon.
    float64 decisionMakingLeastDistance;

    // Lateral distance from the ego vehicle center to the left road boundary in Frenet coordinates (positive value)
    // [m]. Derived at construction as 1.5 x road_half_width, assuming the ego vehicle travels in the center of the
    // right lane on a two-lane road.
    const float64 leftBoundaryDistance =
        static_cast<float64>(decisionConfigReader->getPNCMap().road_half_width_ * 1.5f);

    // Lateral distance from the ego vehicle center to the right road boundary in Frenet coordinates (positive value
    // toward the boundary) [m]. Derived at construction as 0.5 x road_half_width, under the same assumption as
    // leftBoundaryDistance.
    const float64 rightBoundaryDistance =
        static_cast<float64>(decisionConfigReader->getPNCMap().road_half_width_ * 0.5f);

    // Number of path points that constitute the "lead" zone ahead of the ego vehicle [-].
    // Clamped to [LEADPTMINNUM, LEADPTMAXNUM]. Used to scale #decisionMakingLeastDistance with respect to the ego speed
    // so that the decision horizon is proportional to the configured local path length.
    const uint32 decisionMakingLeadPoint =
        std::max(std::min(decisionConfigReader->getLocalPath().path_size_ - 50U, static_cast<uint32>(LEADPTMAXNUM)),
                 static_cast<uint32>(LEADPTMINNUM));

    // Longitudinal extent of the reference line in front of the ego vehicle, computed as reference_line_front_size x
    // segment_len [m]. TPs with a longitudinal distance exceeding this value are outside the planned reference line and
    // are therefore excluded from decision making.
    const float64 referenceLineEndDistance = static_cast<float64>(decisionConfigReader->getReferenceLine().front_size_ *
                                                                  decisionConfigReader->getPNCMap().segment_len_);

    /// Temporary SLPoint built for each TP inside the loop; pushed into decisionPoints when valid.
    SLPoint p{};

    /* Update decisionMakingLeastDistance for this cycle.
       It is computed as ego longitudinal speed (dS/dt) multiplied by the lead point count,
       which approximates the distance the ego vehicle travels during its reaction window.
       A minimum of DMMINLENGTH is enforced so that low-speed scenarios retain a safe planning horizon. */
    decisionMakingLeastDistance = static_cast<float64>(
        std::max(egoCarInfo->getDsDt() * decisionMakingLeadPoint, static_cast<float64>(DMMINLENGTH)));

    // Core: compute decision points based on the traffic participant information and the ego car information
    for (const auto& tpInfo : tpInfoList)
    {
      /* Longitudinal separation between the TP and the ego vehicle in Frenet coordinates.
         Positive value means the TP is ahead of the ego vehicle. */
      const float64 distanceToEgoCar = tpInfo->getS() - egoCarInfo->getS();

      /* Longitudinal range filter: skip TPs that are
         - beyond the reference line end (too far ahead to act on), or
         - more than decisionMakingLeastDistance behind the ego vehicle (already passed). */
      if (distanceToEgoCar > referenceLineEndDistance || distanceToEgoCar < -decisionMakingLeastDistance)
      {
        continue;
      }
      // Tp is inside the corridor boundary or called road
      else if (tpInfo->getL() < leftBoundaryDistance && tpInfo->getL() > rightBoundaryDistance)
      {
        /* Longitudinal speed threshold: a TP is treated as quasi-static when its forward
           speed is less than half the ego speed. This avoids false decisions against TPs
           that are moving nearly as fast as the ego vehicle. */
        const float64 tpLongitudinalSpeedThreshold = egoCarInfo->getDsDt() / 2.0f;

        // consider tp which is nearly static: no lateral movement and low longitudinal speed, which may cause collision
        // if ego car is close to it
        if (std::fabs(tpInfo->getDlDt()) < MINSPEED && tpInfo->getDsDt() < tpLongitudinalSpeedThreshold)
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
             plus one safety margin on each side (2 × safe_dis_lat). */
          if (tpDistanceToLeftBoundary >
              egoCarInfo->getVehicleWidth() + decisionConfigReader->getDecision().safe_dis_lat_ * 2.0)
          {
            /* Target lateral position is the midpoint between the left road boundary and the TP's
               left bounding-box edge, placing the ego vehicle in the center of the available gap. */
            p.l = leftBoundaryDistance + tpBoundingBoxLeftBoundary / 2.0;
            p.type = SLPointType::DECISION_LEFT_OVERTAKE;
            decisionPoints.emplace_back(p);
          }
          /* Right overtake feasibility: same clearance check on the right side. Only evaluated when
             left overtake is not feasible (strict priority: left > right > stop). */
          else if (tpDistanceToRightBoundary >
                   egoCarInfo->getVehicleWidth() + decisionConfigReader->getDecision().safe_dis_lat_ * 2.0)
          {
            /* Target lateral position is the midpoint between the right road boundary and the TP's
               right bounding-box edge. */
            p.l = rightBoundaryDistance + tpBoundingBoxRightBoundary / 2.0;
            p.type = SLPointType::DECISION_RIGHT_OVERTAKE;
            decisionPoints.emplace_back(p);
          }
          /* Stop decision: neither side provides sufficient clearance. The ego vehicle is commanded
             to stop safe_dis_lon metres ahead of the predicted encounter point and wait for the TP
             to clear the path. Processing subsequent TPs is aborted because a stop supersedes all
             further decisions. */
          else
          {
            p.l = 0.0f;
            p.s = p.s - decisionConfigReader->getDecision().safe_dis_lon_;
            p.type = SLPointType::DECISION_STOP;
            RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                        "Traffic participant ID %d cannot be overtaken and is blocking the path. "
                        "Decide to stop at s = %.2f m, l = %.2f m.",
                        tpInfo->getVehicleID(), p.s, p.l);
            decisionPoints.emplace_back(p);
            // break to avoid processing following tps.
            break;
          }
        }
        // tp is moving
        else
        {
          // Todo: add tp is moving decision logic.
        }
      }
      // Tp is not inside the corridor boundary or called road
      else
      {
        // do nothing.
      }
    }

    if (decisionPoints.empty())
    {
      RCLCPP_INFO(rclcpp::get_logger("decision_center"), "No decision point generated in this cycle");
      return;
    }

    /* Prepend a DECISION_START waypoint located decisionMakingLeastDistance before the first
       decision point. This gives the local planner a smooth lead-in ramp before the first lateral
       or longitudinal maneuver takes effect. */
    SLPoint decisionStartPoint{};
    decisionStartPoint.s = decisionPoints.front().s - decisionMakingLeastDistance;
    decisionStartPoint.l = 0.0;
    decisionStartPoint.type = SLPointType::DECISION_START;
    decisionPoints.emplace(decisionPoints.begin(), decisionStartPoint);

    /* Append a DECISION_END waypoint only when the last action is NOT a stop.
       For a stop decision the vehicle does not continue past the stop point, so no
       end marker is needed. For overtake decisions the end point signals where the ego
       vehicle may return to the reference line center (l = 0). */
    if (decisionPoints.back().type != SLPointType::DECISION_STOP)
    {
      SLPoint decisionEndPoint{};
      decisionEndPoint.s = decisionPoints.back().s + decisionMakingLeastDistance;
      decisionEndPoint.l = 0.0;
      decisionEndPoint.type = SLPointType::DECISION_END;
      decisionPoints.emplace_back(decisionEndPoint);
    }
    else
    {
      // do nothing.
    }
  }

  void DecisionCenter::decisionInitialize()
  {
    // Clear all decision points produced in the previous cycle to prevent stale data from
    // propagating into the current planning iteration.
    decisionPoints.clear();
  }

} // namespace Planning
