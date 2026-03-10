#include "local_path_planner.h"

namespace Planning
{
  LocalPathPlanner::LocalPathPlanner()
  {
    RCLCPP_INFO(rclcpp::get_logger("local_path"), "LocalPathPlanner initialized");

    // Read config file for local path planner component
    localPathConfigReader = std::make_unique<ConfigReader>();
    localPathConfigReader->readLocalPathConfig();

    // Initialize local path smoother
    localPathSmoother = std::make_shared<LocalPathSmoother>();
  }
  base_msgs::msg::LocalPath LocalPathPlanner::generateLocalPath(const base_msgs::msg::Referline& referenceLine,
                                                                const std::shared_ptr<DecisionCenter>& decision,
                                                                const std::shared_ptr<VehicleInfoBase>& egoCarInfo)
  {
    // iniitialize local path before generating new one
    initializeLocalPath();

    // calculate s, l value for waypoints on local path
    float64 wayPoint_s =
        egoCarInfo->getS(); // temp s value for waypoints, initialized as the s value of the ego vehicle
    base_msgs::msg::LocalPathPoint localPathPointTmp;
    for (uint i = 0U; i < localPathConfigReader->getLocalPath().path_size_; i++)
    {
      // start point of local path
      wayPoint_s += egoCarInfo->getDsDt(); // local path start point should be planned next cycle
      if (wayPoint_s > referenceLine.refer_line.back().rs)
      {
        break; // break if the local path point exceeds the reference line
      }

      // initialize localPathPointTmp
      localPathPointTmp.s = wayPoint_s;
      // initialize ds/dt of local path point as the current speed of ego vehicle
      localPathPointTmp.ds_dt = egoCarInfo->getDsDt();
      // initialize dds/dt of local path point as the current acceleration of ego vehicle
      localPathPointTmp.dds_dt = egoCarInfo->getDdsDt();

      // initialize l, dl/ds and ddl/ds of local path point as 0, which will be updated later based on the decision pts
      localPathPointTmp.l = 0.0;
      localPathPointTmp.dl_ds = 0.0;
      localPathPointTmp.ddl_ds = 0.0;

      // compute l and dl/ds for localPathPointTmp
      const uint8 decisionPointSize = static_cast<uint8>(decision->getDecisionPoints().size());
      for (uint8 j = 0; j < decisionPointSize - 1U; j++)
      {
        // check if the local path point is between two decision points
        const float64 segmentStart_s = decision->getDecisionPoints().at(j).s;
        const float64 segmentStart_l = decision->getDecisionPoints().at(j).l;
        const float64 segmentStart_dlds = 0.0;
        const float64 segmentStart_ddlds = 0.0;
        const float64 segmentEnd_s = decision->getDecisionPoints().at(j + 1).s;
        const float64 segmentEnd_l = decision->getDecisionPoints().at(j + 1).l;
        const float64 segmentEnd_dlds = 0.0;
        const float64 segmentEnd_ddlds = 0.0;
        if (wayPoint_s >= segmentStart_s && wayPoint_s < segmentEnd_s)
        {
          // utilize 5th-order polynomial to compute l, dl/ds and ddl/ds for localPathPointTmp
          const Eigen::Vector<float64, 6U> coeffsA = PolynomialCurve::computeQuinticPolynomialCoefficients(
              segmentStart_s, segmentStart_l, segmentStart_dlds, segmentStart_ddlds, segmentEnd_s, segmentEnd_l,
              segmentEnd_dlds, segmentEnd_ddlds);
          localPathPointTmp.l = coeffsA(0) + coeffsA(1) * wayPoint_s + coeffsA(2) * std::pow(wayPoint_s, 2.0) +
                                coeffsA(3) * std::pow(wayPoint_s, 3.0) + coeffsA(4) * std::pow(wayPoint_s, 4.0) +
                                coeffsA(5) * std::pow(wayPoint_s, 5.0);
          localPathPointTmp.dl_ds =
              coeffsA(1) + 2.0 * coeffsA(2) * wayPoint_s + 3.0 * coeffsA(3) * std::pow(wayPoint_s, 2.0) +
              4.0 * coeffsA(4) * std::pow(wayPoint_s, 3.0) + 5.0 * coeffsA(5) * std::pow(wayPoint_s, 4.0);
          localPathPointTmp.ddl_ds = 2.0 * coeffsA(2) + 6.0 * coeffsA(3) * wayPoint_s +
                                     12.0 * coeffsA(4) * std::pow(wayPoint_s, 2.0) +
                                     20.0 * coeffsA(5) * std::pow(wayPoint_s, 3.0);
        }
      }
      localPath.local_path.emplace_back(localPathPointTmp);
    }

    // smooth local path in Frenet coordinates
    localPathSmoother->smoothLocalPath(localPath);

    // transform local path from Frenet coordinates to Cartesian coordinates
    tf2::Quaternion qtn;
    for (auto& pt : localPath.local_path)
    {
      // step 1: calculate projected point parameters for the local path point on reference line
      const float64 rs = pt.s;
      const int16 matchPointIndex = Curve::findMatchPointIndex(referenceLine, rs);
      const float64 rx = referenceLine.refer_line[matchPointIndex].pose.pose.position.x;
      const float64 ry = referenceLine.refer_line[matchPointIndex].pose.pose.position.y;
      const float64 rtheta = referenceLine.refer_line[matchPointIndex].rtheta;
      const float64 rkappa = referenceLine.refer_line[matchPointIndex].rkappa;
      const float64 rdkappa = referenceLine.refer_line[matchPointIndex].rdkappa;

      // step 2: transform local path point from Frenet coordinates to Cartesian coordinates
      CartesianState cartesianState{};
      const FrenetState frenetState{ pt.s, pt.ds_dt, pt.dds_dt, pt.l, pt.dl_ds, pt.dl_dt, pt.ddl_ds, pt.ddl_dt };
      const ProjectedPointInfo projectedPoint{ rs, rx, ry, rtheta, rkappa, rdkappa };
      Curve::FrenetToCartesian(frenetState, projectedPoint, cartesianState);
      pt.pose.header = localPath.header;
      pt.pose.pose.position.x = cartesianState.x;
      pt.pose.pose.position.y = cartesianState.y;
      pt.theta = cartesianState.theta;
      pt.kappa = cartesianState.curvature;

      qtn.setRPY(0.0, 0.0, cartesianState.theta);
      pt.pose.pose.orientation.x = qtn.getX();
      pt.pose.pose.orientation.y = qtn.getY();
      pt.pose.pose.orientation.z = qtn.getZ();
      pt.pose.pose.orientation.w = qtn.getW();
    }

    // calculate projected point parameters for local path
    Curve::calculateProjectedPointParameters(localPath);

    RCLCPP_INFO(rclcpp::get_logger("local_path"), "Local path generated with %zu points", localPath.local_path.size());

    return localPath;
  }

  nav_msgs::msg::Path LocalPathPlanner::localPathToRviz()
  {
    localPathRviz.header = localPath.header;
    localPathRviz.poses.clear();

    geometry_msgs::msg::PoseStamped pt_tmp;
    for (const auto& pt : localPath.local_path)
    {
      pt_tmp.header = localPathRviz.header;
      pt_tmp.pose = pt.pose.pose;
      localPathRviz.poses.emplace_back(pt_tmp);
    }
    return localPathRviz;
  }

  void LocalPathPlanner::initializeLocalPath()
  {
    localPath.header.frame_id = localPathConfigReader->getPNCMap().frame_; // set the frame id for local path
    localPath.header.stamp = rclcpp::Clock().now();                        // set the timestamp for local path
    localPath.local_path.clear(); // clear the local path points before generating new ones
  }
} // namespace Planning