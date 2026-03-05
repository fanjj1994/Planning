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
                "TP Frenet state: s = %.2f, ds/dt = %.2f, dds/dt = %.2f, l = %.2f, dl/ds = %.4f, dl/dt = %.2f, "
                "ddl/ds = %.6f, ddl/dt = %.2f",
                frenetState.s, frenetState.ds_dt, frenetState.dds_dt, frenetState.l, frenetState.dl_ds,
                frenetState.dl_dt, frenetState.ddl_ds, frenetState.ddl_dt);
  }

} // namespace Planning
