#include "pnc_map_creator_sturn.h"

namespace Planning
{
  PNCMapCreatorSturn::PNCMapCreatorSturn() // s弯道地图
  {
    RCLCPP_INFO(rclcpp::get_logger("pnc_map"), "pnc_map_creator created");

    //读取配置文件，并给参数赋值
    pnc_map_config_ = std::make_unique<ConfigReader>();
    pnc_map_config_->read_pnc_map_config();
    map_type_ = static_cast<int>(PNCMapType::STURN);

    //地图坐标起点
    p_mid_.x = -3.0;
    p_mid_.y = pnc_map_config_->pnc_map().road_half_width_ / 2.0;

    //长度步长与角度步长
    len_step_ = pnc_map_config_->pnc_map().segment_len_;
    theta_step_ = 0.01;

    //初始化地图
    init_pnc_map();

  }
  PNCMap PNCMapCreatorSturn::create_pnc_map() // 生成地图
  {
    draw_straight_x(pnc_map_config_->pnc_map().road_length_ / 3.0,1.0);
    draw_arc(M_PI_2,1.0);
    draw_arc(M_PI_2,-1.0);
    if(pnc_map_.midline.points.size()%2 == 1)
    {
      pnc_map_.midline.points.pop_back();
    }
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.midline);
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.left_boundary);
    pnc_map_markerarray_.markers.emplace_back(pnc_map_.right_boundary);
    RCLCPP_INFO(rclcpp::get_logger("pnc_map"),"pnc_map created. midline points : %ld",pnc_map_.midline.points.size());
    return pnc_map_;
  }
  void PNCMapCreatorSturn::init_pnc_map() //初始化地图
  {
    pnc_map_.header.frame_id = pnc_map_config_->pnc_map().frame_;
    pnc_map_.header.stamp = rclcpp::Clock().now();
    pnc_map_.road_half_width = pnc_map_config_->pnc_map().road_half_width_;

    //中心线格式
    pnc_map_.midline.header = pnc_map_.header;
    pnc_map_.midline.ns = "pnc_map";
    pnc_map_.midline.id = 0;
    pnc_map_.midline.action = Marker::ADD;
    pnc_map_.midline.type = Marker::LINE_LIST;
    pnc_map_.midline.scale.x = 0.05;
    pnc_map_.midline.color.a = 1.0;
    pnc_map_.midline.color.r = 0.7;
    pnc_map_.midline.color.g = 0.7;
    pnc_map_.midline.color.b = 0.0;
    pnc_map_.midline.lifetime = rclcpp::Duration::max();
    pnc_map_.midline.frame_locked = true;

    //左边界格式
    pnc_map_.left_boundary = pnc_map_.midline;
    pnc_map_.left_boundary.id = 1;
    pnc_map_.left_boundary.type = Marker::LINE_STRIP;
    pnc_map_.left_boundary.color.r = 1.0;
    pnc_map_.left_boundary.color.g = 1.0;
    pnc_map_.left_boundary.color.b = 1.0;
    
    //右边界格式
    pnc_map_.right_boundary = pnc_map_.left_boundary;
    pnc_map_.right_boundary.id = 2;
    pnc_map_.right_boundary.color.r = 1.0;
    pnc_map_.right_boundary.color.g = 1.0;
    pnc_map_.right_boundary.color.b = 1.0;

  }
  void PNCMapCreatorSturn::draw_straight_x(const double &length, const double &plus_flag, const double &ratio) //绘制x方向直道
  {
    double len_tmp = 0.0;
    while(len_tmp < length)
    {
      pl_.x = p_mid_.x;
      pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_;
      pr_.x = p_mid_.x;
      pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_;

      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      len_tmp += len_step_ * ratio;
      p_mid_.x += len_step_ * ratio * plus_flag;
    }
  }
  void PNCMapCreatorSturn::draw_arc(const double &angle, const double &plus_flag, const double &ratio)//绘制弧线
  {
    double theta_tmp =0.0;
    while(theta_tmp < angle)
    {
      pl_.x = p_mid_.x - pnc_map_config_->pnc_map().road_half_width_ * std::sin(theta_current_);
      pl_.y = p_mid_.y + pnc_map_config_->pnc_map().road_half_width_ * std::cos(theta_current_);
      pr_.x = p_mid_.x + pnc_map_config_->pnc_map().road_half_width_ * std::sin(theta_current_);
      pr_.y = p_mid_.y - pnc_map_config_->pnc_map().road_half_width_ * std::cos(theta_current_);

      pnc_map_.midline.points.emplace_back(p_mid_);
      pnc_map_.left_boundary.points.emplace_back(pl_);
      pnc_map_.right_boundary.points.emplace_back(pr_);

      double step_x = len_step_ * std::cos(theta_current_);
      double step_y = len_step_ * std::sin(theta_current_);

      p_mid_.x += step_x;
      p_mid_.y += step_y;
      
      theta_tmp += theta_step_ * ratio;
      theta_current_ += theta_step_ * plus_flag * ratio;
    }
  }
} // namespace Planning
