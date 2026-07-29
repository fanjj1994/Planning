#ifndef OBS_MOVE_CMD_H_
#define OBS_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include "config_reader.h"
#include "obs_car_info.h"

namespace Planning
{
  using namespace std::chrono_literals;
  using geometry_msgs::msg::TransformStamped;
  using tf2_ros::TransformBroadcaster;

  struct ObsParam // 障碍物参数
  {
    std::shared_ptr<VehicleBase> obs_;                      // 障碍物
    std::shared_ptr<TransformBroadcaster> obs_broadcaster_; // 坐标广播器
    double pos_x_ = 0.0;
    double pos_y_ = 0.0;
    double theta_ = 0.0;
    double speed_ = 0.0;
  };

  class ObsMoveCmd : public rclcpp::Node // 障碍车运动指令
  {
  public:
    ObsMoveCmd();

  private:
    void obs_broadcaster_tf(); // 广播障碍物坐标变换

  private:
    std::unique_ptr<ConfigReader> obs_move_cmd_config_; // 配置
    std::vector<ObsParam> obses_param;                  // 障碍物参数合集
    rclcpp::TimerBase::SharedPtr timer_;                // 定时器
  };
} // namespace Planning
#endif // OBS_MOVE_CMD_H_
