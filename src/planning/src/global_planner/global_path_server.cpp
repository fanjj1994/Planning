#include "global_path_server.h"

namespace Planning
{
  GlobalPathServer::GlobalPathServer() : Node("global_path_server_node")// 全局路径服务器
  {
    RCLCPP_INFO(this->get_logger(), "global_path_server_node created");

    //创建全局路径的两个发布器
    global_path_pub_ = this->create_publisher<Path>("global_path",10);
    global_path_rviz_pub_ = this->create_publisher<Marker>("global_path_rviz",10);

    //创建全局路径服务器
    global_path_server_ = this->create_service<GlobalPathService>(
      "global_path_server",
      std::bind(&GlobalPathServer::response_global_path_callback,this,_1,_2)
    );
  }

  void GlobalPathServer::response_global_path_callback(const std::shared_ptr<GlobalPathService::Request> request,
                                                       const std::shared_ptr<GlobalPathService::Response> response)//全局回调函数
  {
    //接收请求，多态
    switch (request->global_planner_type)
    {
    case static_cast<int>(GlobalPlannerType::NORMAL):
      global_path_base_ = std::make_shared<GlobalPlannerNormal>();
      break;
    
    default:
      RCLCPP_WARN(this->get_logger(), "Invaild global planner type!");
      return;
    }
    //判断请求是否为空
    if(request->pnc_map.midline.points.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "pnc_map empty, global path can not be created");
      return;
    }

    //搜索并响应全局路径
    const auto global_path = global_path_base_->search_global_path(request->pnc_map);
    response->global_path = global_path;

    //发布全局路径，给局部规划用
    //因为只发布一次，并且path中没有frame_locked，所以无法固定在rviz中
    global_path_pub_->publish(global_path);
    RCLCPP_INFO(this->get_logger(),"global_path published");

    //发布用于rviz显示的全局路径
    const auto global_path_rviz = path2marker(global_path);
    global_path_rviz_pub_->publish(global_path_rviz);
    RCLCPP_INFO(this->get_logger(),"global_path for rviz published");
  }

  Marker GlobalPathServer::path2marker(const Path &path)//path转marker
  {
    Marker path_rviz_;
    path_rviz_.header = path.header;
    path_rviz_.ns = "global_path";
    path_rviz_.id = 0;
    path_rviz_.action = Marker::ADD;
    path_rviz_.type = Marker::LINE_STRIP;
    path_rviz_.scale.x = 0.05;
    path_rviz_.color.a = 1.0;
    path_rviz_.color.r = 0.8;
    path_rviz_.color.g = 0.0;
    path_rviz_.color.b = 0.0;
    path_rviz_.lifetime = rclcpp::Duration::max();
    path_rviz_.frame_locked = true;

    Point p_tmp;
    for(const auto &pose : path.poses)
    {
      p_tmp.x = pose.pose.position.x;
      p_tmp.y = pose.pose.position.y;
      path_rviz_.points.emplace_back(p_tmp);
    }

    return path_rviz_;
  }

} // namespace Planning 


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::GlobalPathServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
