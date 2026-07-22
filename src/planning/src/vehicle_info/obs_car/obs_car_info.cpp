#include "obs_car_info.h"

namespace Planning
{
  ObsCar::ObsCar(const int  &id)//障碍物车辆信息
  {
    RCLCPP_INFO(rclcpp::get_logger("vehicle"),"obs_car created");

    // 读取配置
    vehicle_config_ = std::make_unique<ConfigReader>();
    vehicle_config_ -> read_vehicles_config();

    // 更新基本属性
    child_frame_ = vehicle_config_->obs_pair()[id].frame_;
    length_ = vehicle_config_->obs_pair()[id].length_;
    width_ = vehicle_config_->obs_pair()[id].width_;
    theta_ = vehicle_config_->obs_pair()[id].pose_theta_;
    speed_ = vehicle_config_->obs_pair()[id].speed_ori_;
    id_ = id;

    // 初始化定位点
    tf2::Quaternion qtn;
    qtn.setRPY(0.0,0.0,theta_);
    loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
    loc_point_.header.stamp = rclcpp::Clock().now();
    loc_point_.pose.position.x = vehicle_config_->obs_pair()[id].pose_x_;
    loc_point_.pose.position.y = vehicle_config_->obs_pair()[id].pose_y_;
    loc_point_.pose.position.z = 0.0;
    loc_point_.pose.orientation.x = qtn.getX();
    loc_point_.pose.orientation.y = qtn.getY();
    loc_point_.pose.orientation.z = qtn.getZ();
    loc_point_.pose.orientation.w = qtn.getW();
  }
}//namespace Planning
