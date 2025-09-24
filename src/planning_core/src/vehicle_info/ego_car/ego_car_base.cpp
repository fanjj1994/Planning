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

} // namespace Planning