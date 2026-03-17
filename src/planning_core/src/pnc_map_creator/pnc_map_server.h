#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "base_msgs/srv/pnc_map_service.hpp"
#include "pnc_map_creator_straight.h"
#include "pnc_map_creator_sturn.h"
#include "common_type.h"

namespace Planning
{
  class PNCMapServer : public rclcpp::Node
  {
  public:
    PNCMapServer();
    PNCMapServer(const PNCMapServer &) = delete;
    ~PNCMapServer() = default;

  private:
    // map creator
    std::shared_ptr<PNCMapCreatorBase> mapCreator;

    // map publisher
    rclcpp::Publisher<base_msgs::msg::PNCMap>::SharedPtr mapPublisher;

    // map markerarray publisher (rviz display)
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mapRvizPublisher;

    // map server
    rclcpp::Service<base_msgs::srv::PNCMapService>::SharedPtr mapServer;

    void responsePNCMapCallBack(const std::shared_ptr<base_msgs::srv::PNCMapService::Request> request,
                                const std::shared_ptr<base_msgs::srv::PNCMapService::Response> response);
  };
} // namespace Planning
#endif // ! PNC_MAP_SERVER_H_
