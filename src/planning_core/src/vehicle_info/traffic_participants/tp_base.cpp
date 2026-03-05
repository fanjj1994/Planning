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

} // namespace Planning