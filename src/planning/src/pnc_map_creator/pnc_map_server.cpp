#include "pnc_map_server.h"

namespace Planning
{
  PNCMapServer::PNCMapServer() : Node ("pnc_map_server_node")
  {
    RCLCPP_INFO(this->get_logger(),"pnc_map_server_node created");

    // 地图发布器
    map_pub_ = this->create_publisher<PNCMap>("pnc_map",10);//ros2发布器创建模板
    map_rviz_pub_ =this->create_publisher<MarkerArray>("pnc_map_markerarray",10);

    // 地图服务器
    map_server_ = this->create_service<PNCMapService>(
      "pnc_map_server",
      std::bind(&PNCMapServer::response_pnc_map_callback,this,_1,_2)
    ); //ros2服务器创建模板
  }

  //响应并发布地图
  void PNCMapServer::response_pnc_map_callback(const std::shared_ptr<PNCMapService::Request> request,
                                               const std::shared_ptr<PNCMapService::Response> response)
  {
    //接受请求，多态
    switch (request->map_type)
    {
    case static_cast<int>(PNCMapType::STRAIGHT):
      map_creator_ = std::make_shared<PNCMapCreatorStraight>();
      break;
    case static_cast<int>(PNCMapType::STURN):
      map_creator_ = std::make_shared<PNCMapCreatorSturn>();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Invaild map type!");
      return;
    }

    //创建并响应地图
    const auto pnc_map = map_creator_->create_pnc_map();
    response->pnc_map = pnc_map;

    //发布地图，planning node 用
    map_pub_->publish(pnc_map);
    RCLCPP_INFO(this->get_logger(),"pnc_map published");

    //发布用于rviz显示的地图
    const auto pnc_map_markerarray = map_creator_->pnc_map_markerarray();
    map_rviz_pub_->publish(pnc_map_markerarray);
    RCLCPP_INFO(this->get_logger(),"pnc_map for rviz published");
  }
} // namespace Planning

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Planning::PNCMapServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
