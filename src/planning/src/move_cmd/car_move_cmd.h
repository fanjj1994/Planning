#ifndef CAR_MOVE_CMD_H_
#define CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/msg/local_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "config_reader.h"
#include "main_car_info.h"

namespace Planning
{
  using namespace std::chrono_literals;
  using std::placeholders::_1;

  using base_msgs::msg::LocalTrajectory;
  using geometry_msgs::msg::TransformStamped;
  using tf2_ros::TransformBroadcaster;

  struct car_param
  {
    double pos_x_ = 0.0; // 位置X
    double pos_y_ = 0.0; // 位置Y
    double theta_ = 0.0; // 航向角
    double speed_ = 0.0; // 速度
  };

  class CarMoveCmd : public rclcpp::Node // 主车运动指令
  {
  public:
    CarMoveCmd();

  private:
    void car_broadcast_tf(const LocalTrajectory::SharedPtr trajectory); // 广播主车坐标变换

  private:
    std::unique_ptr<ConfigReader> move_cmd_config_;                         // 配置
    std::shared_ptr<TransformBroadcaster> broadcaster_;                     // 广播方
    rclcpp::Subscription<LocalTrajectory>::SharedPtr local_trajectory_sub_; // 轨迹订阅方
    std::shared_ptr<VehicleBase> car_;                                      // 主车对象，仅用于初始化
    car_param car_param_;                                                   // 主车参数
  };
} // namespace Planning
#endif // CAR_MOVE_CMD_H_
