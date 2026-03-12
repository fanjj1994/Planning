#include "tp_base.h"

namespace Planning
{
  TP::TP(const uint8 id)
  {
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "TP is initialized");

    // Read Config
    vehicleInfoConfigReader = std::make_unique<ConfigReader>();
    vehicleInfoConfigReader->readVehiclesConfig();

    // Initialize basic property for TP from config reader
    vehicleChildFrame = vehicleInfoConfigReader->getVehiclePairs().at(id).frame_;
    vehicleLength = static_cast<float64>(vehicleInfoConfigReader->getVehiclePairs().at(id).length_);
    vehicleWidth = static_cast<float64>(vehicleInfoConfigReader->getVehiclePairs().at(id).width_);
    vehicleID = vehicleInfoConfigReader->getVehiclePairs().at(id).id_;
    vehicleTheta = static_cast<float64>(vehicleInfoConfigReader->getVehiclePairs().at(id).pose_theta_);
    vehicleVelocity = static_cast<float64>(vehicleInfoConfigReader->getVehiclePairs().at(id).speed_init_);

    // initialize tp cars' pose
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, vehicleTheta);
    vehiclePose.header.frame_id = vehicleInfoConfigReader->getPNCMap().frame_;
    vehiclePose.header.stamp = rclcpp::Clock().now();
    vehiclePose.pose.position.x = static_cast<float64>(vehicleInfoConfigReader->getVehiclePairs().at(id).pose_x_);
    vehiclePose.pose.position.y = static_cast<float64>(vehicleInfoConfigReader->getVehiclePairs().at(id).pose_y_);
    vehiclePose.pose.position.z = 0.0f;
    vehiclePose.pose.orientation.x = qtn.getX();
    vehiclePose.pose.orientation.y = qtn.getY();
    vehiclePose.pose.orientation.z = qtn.getZ();
    vehiclePose.pose.orientation.w = qtn.getW();
  }

  void TP::vehicleCartesianToFrenet(const base_msgs::msg::Referline &referenceline)
  {
    // initialize projected point info struct
    ProjectedPointInfo projectedPoint;

    // initialize Cartesian state struct
    CartesianState cartesianState;
    cartesianState.x = vehiclePose.pose.position.x;
    cartesianState.y = vehiclePose.pose.position.y;
    cartesianState.theta = vehicleTheta;
    cartesianState.speed = vehicleVelocity;
    cartesianState.acceleration = vehicleAcceleration;
    cartesianState.curvature = vehicleKappa;

    // initialize Frenet state struct
    FrenetState frenetState;

    // calculate TP's projected point on the reference line
    Curve::figureOutProjectedPoint(referenceline, vehiclePose, projectedPoint);
    RCLCPP_INFO(rclcpp::get_logger("vehicle"),
                "TP projected point on reference line: rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = "
                "%.4f, rdkappa = %.6f",
                projectedPoint.rs, projectedPoint.rx, projectedPoint.ry, projectedPoint.rtheta, projectedPoint.rkappa,
                projectedPoint.rdkappa);

    // calculate TP's Frenet state based on its Cartesian state and projected point info on the reference line
    Curve::CartesianToFrenet(cartesianState, projectedPoint, frenetState);

    RCLCPP_INFO(rclcpp::get_logger("vehicle"),
                "TP Frenet state on reference line: s = %.2f, ds/dt = %.2f, dds/dt = %.2f, l = %.2f, dl/ds = %.4f, "
                "dl/dt = %.2f, "
                "ddl/ds = %.6f, ddl/dt = %.2f",
                frenetState.s, frenetState.ds_dt, frenetState.dds_dt, frenetState.l, frenetState.dl_ds,
                frenetState.dl_dt, frenetState.ddl_ds, frenetState.ddl_dt);
    // store computed Frenet state into member variables
    s = frenetState.s;
    l = frenetState.l;
    ds_dt = frenetState.ds_dt;
    dl_dt = frenetState.dl_dt;
    dl_ds = frenetState.dl_ds;
    dds_dt = frenetState.dds_dt;
    ddl_dt = frenetState.ddl_dt;
    ddl_ds = frenetState.ddl_ds;
  }

  void TP::vehicleCartesianToFrenet2Path(const base_msgs::msg::LocalPath &localPath,
                                         const base_msgs::msg::Referline &referenceline,
                                         const std::shared_ptr<VehicleInfoBase> &egoCar)
  {
    // compute corresponding index of local path start & end point on reference line, then process should be unified
    // under reference line
    const int16 idxStartInRefLine = Curve::findMatchPointIndex(referenceline, localPath.local_path.front().pose);
    const int16 idxEndInRefLine = Curve::findMatchPointIndex(referenceline, localPath.local_path.back().pose);
    // identify if tps exceeds local path range
    if ((s > referenceline.refer_line[idxEndInRefLine].rs) || (s < referenceline.refer_line[idxStartInRefLine].rs))
    {
      s_2path = s - referenceline.refer_line[idxStartInRefLine].rs;
      ds_dt_2path = ds_dt;
      l_2path = l - egoCar->getL();
      dl_ds_2path = dl_ds;
      dl_dt_2path = dl_dt;
      dds_dt_2path = dds_dt;
      ddl_dt_2path = ddl_dt;
      ddl_ds_2path = ddl_ds;
      RCLCPP_INFO(rclcpp::get_logger("vehicle"), "TP is out of local path range, s_2path = %.2f", s_2path);
      return;
    }
    else
    {
      // tps is inside local path range
      // initialize projected point info struct
      ProjectedPointInfo projectedPoint;

      // initialize Cartesian state struct
      CartesianState cartesianState;
      cartesianState.x = vehiclePose.pose.position.x;
      cartesianState.y = vehiclePose.pose.position.y;
      cartesianState.theta = vehicleTheta;
      cartesianState.speed = vehicleVelocity;
      cartesianState.acceleration = vehicleAcceleration;
      cartesianState.curvature = vehicleKappa;

      // initialize Frenet state struct
      FrenetStateOnLocalPath frenetStateOnLocalPath;

      // calculate TP's projected point on the local path
      Curve::figureOutProjectedPoint(localPath, vehiclePose, projectedPoint);
      RCLCPP_INFO(rclcpp::get_logger("vehicle"),
                  "TP projected point on local path: rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = "
                  "%.4f, rdkappa = %.6f",
                  projectedPoint.rs, projectedPoint.rx, projectedPoint.ry, projectedPoint.rtheta, projectedPoint.rkappa,
                  projectedPoint.rdkappa);

      // calculate TP's Frenet state based on its Cartesian state and projected point info on the local path
      Curve::CartesianToFrenet(cartesianState, projectedPoint, frenetStateOnLocalPath);

      RCLCPP_INFO(
          rclcpp::get_logger("vehicle"),
          "TP Frenet state on local path: s = %.2f, ds/dt = %.2f, dds/dt = %.2f, l = %.2f, dl/ds = %.4f, dl/dt = %.2f, "
          "ddl/ds = %.6f, ddl/dt = %.2f",
          frenetStateOnLocalPath.s_2path, frenetStateOnLocalPath.ds_dt_2path, frenetStateOnLocalPath.dds_dt_2path,
          frenetStateOnLocalPath.l_2path, frenetStateOnLocalPath.dl_ds_2path, frenetStateOnLocalPath.dl_dt_2path,
          frenetStateOnLocalPath.ddl_ds_2path, frenetStateOnLocalPath.ddl_dt_2path);

      // store computed Frenet state into member variables
      s_2path = frenetStateOnLocalPath.s_2path;
      l_2path = frenetStateOnLocalPath.l_2path;
      ds_dt_2path = frenetStateOnLocalPath.ds_dt_2path;
      dl_dt_2path = frenetStateOnLocalPath.dl_dt_2path;
      dl_ds_2path = frenetStateOnLocalPath.dl_ds_2path;
      dds_dt_2path = frenetStateOnLocalPath.dds_dt_2path;
      ddl_dt_2path = frenetStateOnLocalPath.ddl_dt_2path;
      ddl_ds_2path = frenetStateOnLocalPath.ddl_ds_2path;
    }
  }
} // namespace Planning
