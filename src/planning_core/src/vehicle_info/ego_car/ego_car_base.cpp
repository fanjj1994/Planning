#include "ego_car_base.h"

namespace Planning
{
  EgoCar::EgoCar()
  {
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "EgoCar is initialized");

    // Read Config
    vehicleInfoConfigReader = std::make_unique<ConfigReader>();
    vehicleInfoConfigReader->readVehiclesConfig();

    // initialize basic property for ego car from config reader
    vehicleChildFrame = vehicleInfoConfigReader->getEgoCar().frame_;
    vehicleLength = static_cast<float64>(vehicleInfoConfigReader->getEgoCar().length_);
    vehicleWidth = static_cast<float64>(vehicleInfoConfigReader->getEgoCar().width_);
    vehicleID = vehicleInfoConfigReader->getEgoCar().id_;
    vehicleTheta = static_cast<float64>(vehicleInfoConfigReader->getEgoCar().pose_theta_);
    vehicleVelocity = static_cast<float64>(vehicleInfoConfigReader->getEgoCar().speed_init_);

    // initialize ego car's pose
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, vehicleTheta);
    vehiclePose.header.frame_id = vehicleInfoConfigReader->getPNCMap().frame_;
    vehiclePose.header.stamp = rclcpp::Clock().now();
    vehiclePose.pose.position.x = static_cast<float64>(vehicleInfoConfigReader->getEgoCar().pose_x_);
    vehiclePose.pose.position.y = static_cast<float64>(vehicleInfoConfigReader->getEgoCar().pose_y_);
    vehiclePose.pose.position.z = 0.0;
    vehiclePose.pose.orientation.x = static_cast<float64>(qtn.getX());
    vehiclePose.pose.orientation.y = static_cast<float64>(qtn.getY());
    vehiclePose.pose.orientation.z = static_cast<float64>(qtn.getZ());
    vehiclePose.pose.orientation.w = static_cast<float64>(qtn.getW());
  }

  void EgoCar::vehicleCartesianToFrenet(const base_msgs::msg::Referline &referenceline)
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

    // calculate ego car's projected point on the reference line
    Curve::figureOutProjectedPoint(referenceline, vehiclePose, projectedPoint);
    RCLCPP_INFO(rclcpp::get_logger("vehicle"),
                "EgoCar projected point on reference line: rs = %.2f, rx = %.2f, ry = %.2f, rtheta = %.2f, rkappa = "
                "%.4f, rdkappa = %.6f",
                projectedPoint.rs, projectedPoint.rx, projectedPoint.ry, projectedPoint.rtheta, projectedPoint.rkappa,
                projectedPoint.rdkappa);

    // calculate ego car's Frenet state based on its Cartesian state and projected point info on the reference line
    Curve::CartesianToFrenet(cartesianState, projectedPoint, frenetState);

    RCLCPP_INFO(rclcpp::get_logger("vehicle"),
                "EgoCar Frenet state: s = %.2f, ds/dt = %.2f, dds/dt = %.2f, l = %.2f, dl/ds = %.4f, dl/dt = %.2f, "
                "ddl/ds = %.6f, ddl/dt = %.2f",
                frenetState.s, frenetState.ds_dt, frenetState.dds_dt, frenetState.l, frenetState.dl_ds,
                frenetState.dl_dt, frenetState.ddl_ds, frenetState.ddl_dt);
    // store computed Frenet state into member variables
    // Todo: consider storing the whole FrenetState struct as a member variable if needed in the future.
    s = frenetState.s;
    l = frenetState.l;
    ds_dt = frenetState.ds_dt;
    dl_dt = frenetState.dl_dt;
    dl_ds = frenetState.dl_ds;
    dds_dt = frenetState.dds_dt;
    ddl_dt = frenetState.ddl_dt;
    ddl_ds = frenetState.ddl_ds;
  }

} // namespace Planning
