#include "local_speeds_planner.h"

namespace Planning
{
  LocalSpeedsPlanner::LocalSpeedsPlanner()
  {
    RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "LocalSpeedsPlanner initialized");

    // Read config file for local speeds planner
    localSpeedsPlannerConfigReader = std::make_unique<ConfigReader>();
    localSpeedsPlannerConfigReader->readLocalSpeedsConfig();

    // Initialize local speeds smoother
    localSpeedsSmoother = std::make_shared<LocalSpeedsSmoother>();
  }
  base_msgs::msg::LocalSpeeds LocalSpeedsPlanner::planLocalSpeeds(const std::shared_ptr<DecisionCenter> &decision)
  {
    // iniitialize local speeds before generating new one
    initializeLocalSpeeds();

    // calculate s,t value for waypoints on local speeds
    float64 wayPoint_t = 0.0; // temp t value for waypoints
    base_msgs::msg::LocalSpeedsPoint localSpeedsPointTmp;

    for (uint8 i = 0U; i < localSpeedsPlannerConfigReader->getLocalSpeeds().speeds_size_; i++)
    {
      // look through decision result, identify each segment's start and end status
      wayPoint_t += 1.0; // local speeds planning start point. unit: frame
      if (wayPoint_t > localSpeedsPlannerConfigReader->getLocalSpeeds().speeds_size_)
      {
        break; // break if the local speeds point exceeds the planning time horizon
      }
      localSpeedsPointTmp.t = wayPoint_t;
      // initialize s_2path, ds_dt_2path
      localSpeedsPointTmp.s_2path = localSpeedsPlannerConfigReader->getEgoCar().set_speed_ * wayPoint_t;
      localSpeedsPointTmp.ds_dt_2path = localSpeedsPlannerConfigReader->getEgoCar().set_speed_;
      localSpeedsPointTmp.dds_dt_2path = 0.0; // initialize acceleration to 0

      const int32 decisionPointSize = static_cast<int32>(decision->getSpeedDecisionPoints().size());
      for (int32 j = 0; j < decisionPointSize - 1; j++)
      {
        const float64 segmentStart_t = decision->getSpeedDecisionPoints().at(j).t;
        const float64 segmentStart_s_2path = decision->getSpeedDecisionPoints().at(j).s_2path;
        const float64 segmentStart_ds_dt_2path = decision->getSpeedDecisionPoints().at(j).ds_dt_2path;
        const float64 segmentStart_dds_dt_2path = 0.0;

        const float64 segmentEnd_t = decision->getSpeedDecisionPoints().at(j + 1).t;
        const float64 segmentEnd_s_2path = decision->getSpeedDecisionPoints().at(j + 1).s_2path;
        const float64 segmentEnd_ds_dt_2path = decision->getSpeedDecisionPoints().at(j + 1).ds_dt_2path;
        const float64 segmentEnd_dds_dt_2path = 0.0;

        if ((wayPoint_t >= segmentStart_t) && (wayPoint_t < segmentEnd_t))
        {
          if ((segmentEnd_t == decision->getSpeedDecisionPoints().back().t) &&
              (segmentEnd_s_2path == decision->getSpeedDecisionPoints().back().s_2path))
          {
            // if the local speeds point is in the last segment, directly use 1st order line to connect two points
            const Eigen::Vector2d coeffsA = PolynomialCurve::computeLinearPolynomialCoefficients(
                segmentStart_t, segmentStart_s_2path, segmentEnd_t, segmentEnd_s_2path);
            localSpeedsPointTmp.s_2path = coeffsA(0) + coeffsA(1) * wayPoint_t;
            localSpeedsPointTmp.ds_dt_2path = coeffsA(1);
            localSpeedsPointTmp.dds_dt_2path = 0.0;

            if ((std::fabs(localSpeedsPointTmp.ds_dt_2path) < MINSPEED))
            {
              localSpeedsPointTmp.ds_dt_2path = 0.0;
              localSpeedsPointTmp.dds_dt_2path = 0.0;
            }
          }
          else
          {
            // utilize 5th order polynomial to compute s_2path, ds/dt_2path and dds/dt_2path for localSpeedsPointTmp
            const Eigen::Vector<float64, 6U> coeffsA = PolynomialCurve::computeQuinticPolynomialCoefficients(
                segmentStart_t, segmentStart_s_2path, segmentStart_ds_dt_2path, segmentStart_dds_dt_2path, segmentEnd_t,
                segmentEnd_s_2path, segmentEnd_ds_dt_2path, segmentEnd_dds_dt_2path);
            localSpeedsPointTmp.s_2path =
                coeffsA(0) + coeffsA(1) * wayPoint_t + coeffsA(2) * std::pow(wayPoint_t, 2.0) +
                coeffsA(3) * std::pow(wayPoint_t, 3.0) + coeffsA(4) * std::pow(wayPoint_t, 4.0) +
                coeffsA(5) * std::pow(wayPoint_t, 5.0);
            localSpeedsPointTmp.ds_dt_2path =
                coeffsA(1) + 2.0 * coeffsA(2) * wayPoint_t + 3.0 * coeffsA(3) * std::pow(wayPoint_t, 2.0) +
                4.0 * coeffsA(4) * std::pow(wayPoint_t, 3.0) + 5.0 * coeffsA(5) * std::pow(wayPoint_t, 4.0);
            localSpeedsPointTmp.dds_dt_2path = 2.0 * coeffsA(2) + 6.0 * coeffsA(3) * wayPoint_t +
                                               12.0 * coeffsA(4) * std::pow(wayPoint_t, 2.0) +
                                               20.0 * coeffsA(5) * std::pow(wayPoint_t, 3.0);
          }
        }
      }
      // Simplify local speeds Frenet-Cartesian Transformation by directly using ds/dt, dds/dt as speed and acceleration
      // for local speeds points
      localSpeedsPointTmp.speed = localSpeedsPointTmp.ds_dt_2path;
      localSpeedsPointTmp.acceleration = localSpeedsPointTmp.dds_dt_2path;
      localSpeeds.local_speeds.emplace_back(localSpeedsPointTmp);
    }

    // smooth local speeds
    localSpeedsSmoother->smoothLocalSpeeds(localSpeeds);

    RCLCPP_INFO(rclcpp::get_logger("local_speeds"), "Local speeds generated with %zu points",
                localSpeeds.local_speeds.size());

    return localSpeeds;
  }

  void LocalSpeedsPlanner::initializeLocalSpeeds()
  {
    localSpeeds.header.frame_id =
        localSpeedsPlannerConfigReader->getPNCMap().frame_; // set the frame id for local speeds
    localSpeeds.header.stamp = rclcpp::Clock().now();       // set the timestamp for local speeds
    localSpeeds.local_speeds.clear();                       // clear the local speeds points before generating new ones
  }
} // namespace Planning
