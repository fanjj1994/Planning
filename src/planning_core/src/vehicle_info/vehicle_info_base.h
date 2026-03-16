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

    // Frenet coordinate parameters (on reference line)
    float64 s;
    float64 l;
    float64 ds_dt;
    float64 dl_dt;
    float64 dl_ds;
    float64 dds_dt;
    float64 ddl_dt;
    float64 ddl_ds;

    // Frenet coordinate parameters (on local path)
    float64 s_2path;
    float64 l_2path;
    float64 ds_dt_2path;
    float64 dl_dt_2path;
    float64 dl_ds_2path;
    float64 dds_dt_2path;
    float64 ddl_dt_2path;
    float64 ddl_ds_2path;

    // time parameters
    float64 t0;    // time to start speeds planning
    float64 t;     // current time
    float64 t_in;  // time for tps to cut in ego vehicle's drivable corridor
    float64 t_out; // time for tps to cut out ego vehicle's drivable corridor

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
          ddl_ds(0.0),
          s_2path(0.0),
          l_2path(0.0),
          ds_dt_2path(0.0),
          dl_dt_2path(0.0),
          dl_ds_2path(0.0),
          dds_dt_2path(0.0),
          ddl_dt_2path(0.0),
          ddl_ds_2path(0.0),
          t0(0.0),
          t(0.0),
          t_in(0.0),
          t_out(0.0)

    {
    }
    VehicleInfoBase(const VehicleInfoBase &) = delete;
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

    inline float64 getS2Path() const { return s_2path; }

    inline float64 getL2Path() const { return l_2path; }

    inline float64 getDsDt2Path() const { return ds_dt_2path; }

    inline float64 getDlDt2Path() const { return dl_dt_2path; }

    inline float64 getDlDs2Path() const { return dl_ds_2path; }

    inline float64 getDdsDt2Path() const { return dds_dt_2path; }

    inline float64 getDdlDt2Path() const { return ddl_dt_2path; }

    inline float64 getDdlDs2Path() const { return ddl_ds_2path; }

    inline float64 getT0() const { return t0; }

    inline float64 getT() const { return t; }

    inline float64 getTIn() const { return t_in; }

    inline float64 getTOut() const { return t_out; }

    // update parameters
    inline void updateVehiclePose(const geometry_msgs::msg::PoseStamped &currentVehiclePose)
    {
      vehiclePose = currentVehiclePose;
    }
    inline void updateCartesianInfo(const base_msgs::msg::LocalTrajectoryPoint &point)
    {
      vehicleTheta = point.path_point.theta;
      vehicleKappa = point.path_point.kappa;
      vehicleDKappa = point.path_point.dkappa;
      vehicleVelocity = point.speed_point.speed;
      vehicleAcceleration = point.speed_point.acceleration;
      vehicleDAcceleration = point.speed_point.dacceleration;
    }

    inline void updateT0()
    {
      t0 -= 1.0; // unit: frame
    }

    inline void setTInTOut(const float64 &t_, const float64 &t_in_, const float64 &t_out_)
    {
      t = t_;
      t_in = t_in_;
      t_out = t_out_;
    }

    // Coordinate transformation from Cartesian to Frenet on reference line. Will be implemented in egoCar and
    // Tps respectively.
    virtual void vehicleCartesianToFrenet(const base_msgs::msg::Referline &referenceline) = 0;

    // Coordinate transformation from Cartesian to Frenet on local path. Will be implemented in egoCar and Tps
    // respectively.
    virtual void vehicleCartesianToFrenet2Path(const base_msgs::msg::LocalPath &localPath,
                                               const base_msgs::msg::Referline &referenceline,
                                               const std::shared_ptr<VehicleInfoBase> &egoCar) = 0;
  };
} // namespace Planning
#endif // ! VEHICLE_INFO_BASE_H_
