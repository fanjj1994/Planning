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
  void ObsCar::vehicle_cartesian_to_frenet(const Referline &refer_line)
  {
    double rs,rx,ry,rtheta,rkappa,rdkappa;

    //计算定位点在参考线上的投影点
    Curve::find_projection_point(refer_line,loc_point_,rs,rx,ry,rtheta,rkappa,rdkappa);
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "obs car projection_point : rs= %.2f , rx = %.2f,ry = %.2f , rtheta = %.2f, rkappa= %.2f, rdkappa= %.2f", 
              rs,rx,ry,rtheta,rkappa,rdkappa);

    // 计算定位点在frenet坐标下的参数 
    Curve::cartesian_to_frenet(loc_point_.pose.position.x,
                                loc_point_.pose.position.y,
                              theta_,speed_,acceleration_,kappa_,
                              rs,rx,ry,rtheta,rkappa,rdkappa,
                              s_,ds_dt_,dds_dt_,l_,dl_ds_,dl_dt_,ddl_ds_,ddl_dt_);
  }

  //向路径投影
  void ObsCar::vehicle_cartesian_to_frenet_2path(const LocalPath &local_path, const Referline &refer_line,
                                                 const std::shared_ptr<VehicleBase> &car)
  {
    // 计算路径起点终点在参考线上的下标
    const double path0_index = Curve::find_match_point(refer_line, local_path.local_path[0].pose);
    const double path_end_index = Curve::find_match_point(refer_line,local_path.local_path.back().pose);

    // 当障碍物在参考线上的s值超出路径首位范围时
    if (s_ > refer_line.refer_line[path_end_index].rs || 
        s_ < refer_line.refer_line[path0_index].rs) // 超出路径前端或者后端
    {
      // 直接用参考线投影的值近似代替路径投影
      s_2path_ = s_ - refer_line.refer_line[path0_index].rs;
      ds_dt_2path_ = ds_dt_;
      l_2path_ = l_ - car->l();
      dl_ds_2path_ = dl_ds_;
      dl_dt_2path_ = dl_dt_;
      dds_dt_2path_ = dds_dt_;
      ddl_ds_2path_ = ddl_ds_;
      ddl_dt_2path_ = ddl_dt_;

      return;
    }

    // 计算定位点在路径上的投影点
     double rs,rx,ry,rtheta,rkappa,rdkappa;
    Curve::find_projection_point(local_path,loc_point_,rs,rx,ry,rtheta,rkappa,rdkappa);
    
    // 计算定位点在frenet坐标下的参数
    Curve::cartesian_to_frenet(loc_point_.pose.position.x,
                                loc_point_.pose.position.y,
                              theta_,speed_,acceleration_,kappa_,
                              rs,rx,ry,rtheta,rkappa,rdkappa,
                              s_2path_,ds_dt_2path_,dds_dt_2path_,l_2path_,dl_ds_2path_,dl_dt_2path_,ddl_ds_2path_,ddl_dt_2path_);
  }
} // namespace Planning
