#ifndef VEHICLE_INFO_BASE_H_
#define VEHICLE_INFO_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/referline.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "config_reader.h"
#include "curve.h"

namespace Planning
{
  class VehicleInfoBase
  {
  private:
  protected:
    // vehicle properties
    std::unique_ptr<ConfigReader> vehicleInfoConfigReader;
    std_string vehicleChildFrame; // child frame ID of vehicle
    float64 vehicleLength;        // vehicle length
    float64 vehicleWidth;         // vehicle width
    uint8 vehicleID;              // vehicle ID

    // Cartesian coordinate parameters
    geometry_msgs::msg::PoseStamped vehiclePose; // vehicle pose in map frame
    float64 vehicleTheta;
    float64 vehicleKappa;
    float64 vehicleDKappa;
    float64 vehicleVelocity;
    float64 vehicleAcceleration;
    float64 vehicleDAcceleration;

    // Frenet coordinate parameters
    float64 s;
    float64 l;
    float64 ds_dt;
    float64 dl_dt;
    float64 dl_ds;
    float64 dds_dt;
    float64 ddl_dt;
    float64 ddl_ds;

  public:
    VehicleInfoBase()
        : vehicleChildFrame(""),
          vehicleLength(0.0),
          vehicleWidth(0.0),
          vehicleID(0U),
          vehiclePose{},
          vehicleTheta(0.0),
          vehicleKappa(0.0),
          vehicleDKappa(0.0),
          vehicleVelocity(0.0),
          vehicleAcceleration(0.0),
          vehicleDAcceleration(0.0),
          s(0.0),
          l(0.0),
          ds_dt(0.0),
          dl_dt(0.0),
          dl_ds(0.0),
          dds_dt(0.0),
          ddl_dt(0.0),
          ddl_ds(0.0)
    {
    }
    VehicleInfoBase(const VehicleInfoBase&) = delete;
    virtual ~VehicleInfoBase() = default;

    // inline getters
    inline std_string getVehicleChildFrame() const { return vehicleChildFrame; }

    inline float64 getVehicleLength() const { return vehicleLength; }

    inline float64 getVehicleWidth() const { return vehicleWidth; }

    inline uint8 getVehicleID() const { return vehicleID; }

    inline geometry_msgs::msg::PoseStamped getVehiclePose() const { return vehiclePose; }

    inline float64 getVehicleTheta() const { return vehicleTheta; }

    inline float64 getVehicleKappa() const { return vehicleKappa; }

    inline float64 getVehicleDKappa() const { return vehicleDKappa; }

    inline float64 getVehicleVelocity() const { return vehicleVelocity; }

    inline float64 getVehicleAcceleration() const { return vehicleAcceleration; }

    inline float64 getVehicleDAcceleration() const { return vehicleDAcceleration; }

    inline float64 getS() const { return s; }

    inline float64 getL() const { return l; }

    inline float64 getDsDt() const { return ds_dt; }

    inline float64 getDlDt() const { return dl_dt; }

    inline float64 getDlDs() const { return dl_ds; }

    inline float64 getDdsDt() const { return dds_dt; }

    inline float64 getDdlDt() const { return ddl_dt; }

    inline float64 getDdlDs() const { return ddl_ds; }

    // update parameters
    inline void updateVehiclePose(const geometry_msgs::msg::PoseStamped& currentVehiclePose)
    {
      vehiclePose = currentVehiclePose;
    }
    inline void updateCartesianInfo(const base_msgs::msg::LocalTrajectoryPoint& point)
    {
      vehicleTheta = point.path_point.theta;
      vehicleKappa = point.path_point.kappa;
      vehicleDKappa = point.path_point.dkappa;
      // vehicleVelocity = point.speed_point.speed;
      // vehicleAcceleration = point.speed_point.acceleration;
      // vehicleDAcceleration = point.speed_point.dacceleration;
    }

    // Coordinate transformation from Cartesian to Frenet, which will be implemented in egoCar and Tps respectively.
    virtual void vehicleCartesianToFrenet(const base_msgs::msg::Referline& referenceline) = 0;
  };
} // namespace Planning
#endif // ! VEHICLE_INFO_BASE_H_
