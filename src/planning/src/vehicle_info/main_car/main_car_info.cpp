#include "main_car_info.h"

namespace Planning
{
  MainCar::MainCar()//主车信息
  {
    RCLCPP_INFO(rclcpp::get_logger("vehicle"),"main_car created");

    // 读取配置
    vehicle_config_ = std::make_unique<ConfigReader>();
    vehicle_config_->read_vehicles_config();

    // 更新基本属性
    child_frame_ = vehicle_config_->main_car().frame_;
    length_ = vehicle_config_->main_car().length_;
    width_ = vehicle_config_->main_car().width_;
    theta_ = vehicle_config_->main_car().pose_theta_;
    speed_ = vehicle_config_->main_car().speed_ori_;
    id_ = 0;

    // 初始化定位点
    tf2::Quaternion qtn;
    qtn.setRPY(0.0,0.0,theta_);
    loc_point_.header.frame_id = vehicle_config_->pnc_map().frame_;
    loc_point_.header.stamp = rclcpp::Clock().now();
    loc_point_.pose.position.x = vehicle_config_->main_car().pose_x_;
    loc_point_.pose.position.y = vehicle_config_->main_car().pose_y_;
    loc_point_.pose.position.z = 0.0;
    loc_point_.pose.orientation.x = qtn.getX();
    loc_point_.pose.orientation.y = qtn.getY();
    loc_point_.pose.orientation.z = qtn.getZ();
    loc_point_.pose.orientation.w = qtn.getW();
  }
  void MainCar::vehicle_cartesian_to_frenet(const Referline &refer_line)
  {
    double rs,rx,ry,rtheta,rkappa,rdkappa;

    //计算定位点在参考线上的投影点
    Curve::find_projection_point(refer_line,loc_point_,rs,rx,ry,rtheta,rkappa,rdkappa);
    RCLCPP_INFO(rclcpp::get_logger("vehicle"), "main car_point : rs= %.2f , rx = %.2f,ry = %.2f , rtheta = %.2f, rkappa= %.2f, rdkappa= %.2f", 
              rs,rx,ry,rtheta,rkappa,rdkappa);

    // 计算定位点在frenet坐标下的参数
    Curve::cartesian_to_frenet(loc_point_.pose.position.x,
                                loc_point_.pose.position.y,
                              theta_,speed_,acceleration_,kappa_,
                              rs,rx,ry,rtheta,rkappa,rdkappa,
                              s_,ds_dt_,dds_dt_,l_,dl_ds_,dl_dt_,ddl_ds_,ddl_dt_);
  }

  //向路径投影
  void MainCar::vehicle_cartesian_to_frenet_2path(const LocalPath &local_path, const Referline &refer_line,
                                                  const std::shared_ptr<VehicleBase> &car)
  {
    //主车不需要向路径投影
    (void)local_path;
    (void)refer_line;
    (void)car;
  }
} // namespace Planning
