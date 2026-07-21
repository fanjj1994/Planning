#ifndef VEHICLE_BASE_H_
#define VEHICLE_BASE_H_

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
  using base_msgs::msg::Referline;
  using geometry_msgs::msg::PoseStamped;
  using geometry_msgs::msg::TransformStamped;
  using nav_msgs::msg::Path;

  class VehicleBase // 车辆基类
  {
  public:
    // 更新参数
    inline void update_location(const PoseStamped &loc) { loc_point_ = loc; }

    // 定位点转frenet

    // 基本属性
    inline std::string child_frame() const { return child_frame_; }
    inline double length() const { return length_; }
    inline double width() const { return width_; }
    inline int id() const { return id_; }

    // 笛卡尔参数
    inline PoseStamped loc_point() const { return loc_point_; }
    inline double theta() const { return theta_; }
    inline double kappa() const { return kappa_; }
    inline double speed() const { return speed_; }
    inline double dkappa() const { return dkappa_; }
    inline double acceleration() const { return acceleration_; }
    inline double dacceleration() const { return dacceleration_; }

    // 向参考线投影的frenet参数

    // 向路径投影的frenet参数

    // 时间参数

    // 虚析构
    virtual ~VehicleBase() {}

  protected:
    // 基本属性
    std::unique_ptr<ConfigReader> vehicle_config_; // 配置
    std::string child_frame_;                      // 坐标名
    double length_ = 0.0;                          // 长
    double width_ = 0.0;                           // 宽
    int id_ = 0.0;                                 // 序号

    // 笛卡尔参数
    PoseStamped loc_point_;      // 定位点
    double theta_ = 0.0;         // 航向角
    double kappa_ = 0.0;         // 曲率
    double dkappa_ = 0.0;        // 曲率变化率
    double speed_ = 0.0;         // 速度
    double acceleration_ = 0.0;  // 加速度
    double dacceleration_ = 0.0; // 加加速度

    // 向参考线投影的frenet参数

    // 向路径投影的frenet参数

    // 时间参数
  };
} // namespace Planning
#endif // VEHICLE_BASE_H_
