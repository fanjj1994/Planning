#ifndef TP_MOVE_CMD_H_
#define TP_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include "tp_base.h"

namespace Planning
{
  struct TpParam
  {
    float64 posX;                                                 // unit: m
    float64 posY;                                                 // unit: m
    float64 theta;                                                // unit: rad
    float64 speed;                                                // unit: m/s
    std::shared_ptr<VehicleInfoBase> tpCar;                       // to store the tp car's state information
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster; // to broadcast the transform of tp car
  };
  class TPMoveCmd : public rclcpp::Node
  {
  public:
    TPMoveCmd();
    TPMoveCmd(const TPMoveCmd &other) = delete;
    TPMoveCmd &operator=(const TPMoveCmd &other) = delete;
    ~TPMoveCmd() = default;

  private:
    std::unique_ptr<ConfigReader> configReaderTp;
    std::vector<TpParam> tpCarParams;            // to store the parameters of tp cars for move command
    rclcpp::TimerBase::SharedPtr timerTpMoveCmd; // to control the frequency of tp car move command
    void tpBroadcastTf(); // to broadcast the transform of tp car based on its speed and heading direction
  };
} // namespace Planning
#endif // ! TP_MOVE_CMD_H_
