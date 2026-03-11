#ifndef EGO_CAR_MOVE_CMD_H_
#define EGO_CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "config_reader.h"
#include "ego_car_base.h"

namespace Planning
{
  struct CarParam
  {
    float64 posX;  // unit: m
    float64 posY;  // unit: m
    float64 theta; // unit: rad
    float64 speed; // unit: m/s
  };
  class EgoCarMoveCmd : public rclcpp::Node
  {
  public:
    EgoCarMoveCmd();
    EgoCarMoveCmd(const EgoCarMoveCmd &other) = delete;
    EgoCarMoveCmd &operator=(const EgoCarMoveCmd &other) = delete;
    ~EgoCarMoveCmd() = default;

  private:
    // to broadcast the transform of ego car
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    // to subscribe the local trajectory from planning process
    rclcpp::Subscription<base_msgs::msg::LocalTrajectory>::SharedPtr localTrajectorySubscription;
    // to read config file for ego car move command
    std::unique_ptr<ConfigReader> configReaderCarMoveCmd;
    // to store the ego car's state information
    std::shared_ptr<VehicleInfoBase> egoCar;
    // to store the ego car's parameters for move command
    CarParam carParam;

    // broadcast the transform of ego car based on the received local trajectory
    void carBroadcastTf(const base_msgs::msg::LocalTrajectory::SharedPtr msg);
  };
} // namespace Planning
#endif // ! EGO_CAR_MOVE_CMD_H_
