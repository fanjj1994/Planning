#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "curve.h"

namespace Planning
{
class VehicleInfoBase : public rclcpp::Node
{
private:
protected:
  // vehicle properties
  std::unique_ptr<ConfigReader> vehicleInfoConfigReader;
  std::string vehicleChildFrame{ "" };  // child frame ID of vehicle
  float64 vehicleLength{ 0.0 };         // vehicle length
  float64 vehicleWidth{ 0.0 };          // vehicle width
  uint8 vehicleID{ 0U };                // vehicle ID

  // Cartesian coordinate parameters
  geometry_msgs::msg::PoseStamped vehiclePose{};  // vehicle pose in map frame
  float64 vehicleTheta{ 0.0 };
  float64 vehicleKappa{ 0.0 };
  float64 vehicleDKappa{ 0.0 };
  float64 vehicleVelocity{ 0.0 };
  float64 vehicleAcceleration{ 0.0 };
  float64 vehicleDAcceleration{ 0.0 };

  // Frenet coordinate parameters

public:
  VehicleInfoBase();
  VehicleInfoBase(const VehicleInfoBase&) = delete;
  virtual ~VehicleInfoBase() = default;

  // inline getters
  inline std::string getVehicleChildFrame() const
  {
    return vehicleChildFrame;
  }

  inline float64 getVehicleLength() const
  {
    return vehicleLength;
  }

  inline float64 getVehicleWidth() const
  {
    return vehicleWidth;
  }

  inline uint8 getVehicleID() const
  {
    return vehicleID;
  }

  inline geometry_msgs::msg::PoseStamped getVehiclePose() const
  {
    return vehiclePose;
  }

  inline float64 getVehicleTheta() const
  {
    return vehicleTheta;
  }

  inline float64 getVehicleKappa() const
  {
    return vehicleKappa;
  }

  inline float64 getVehicleDKappa() const
  {
    return vehicleDKappa;
  }

  inline float64 getVehicleVelocity() const
  {
    return vehicleVelocity;
  }

  inline float64 getVehicleAcceleration() const
  {
    return vehicleAcceleration;
  }

  inline float64 getVehicleDAcceleration() const
  {
    return vehicleDAcceleration;
  }

  // update parameters
  inline void updateVehiclePose(const geometry_msgs::msg::PoseStamped& currentVehiclePose)
  {
    vehiclePose = currentVehiclePose;
  }
};
}  // namespace Planning
#endif  // ! VEHICLE_INFO_BASE_H_
