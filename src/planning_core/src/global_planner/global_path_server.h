#ifndef GLOABAL_PATH_SERVER_H_
#define GLOABAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "global_planner_normal.h"

#include "base_msgs/srv/global_path_service.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace Planning {
  class GlobalPathServer : public rclcpp::Node
  {
  public:
    GlobalPathServer();
    GlobalPathServer(const GlobalPathServer &) = delete;
    ~GlobalPathServer() = default;

  private:
    // global path creator
    std::shared_ptr<GlobalPlannerBase> globalPlannerCreator;

    // global path publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr globalPathPublisher;

    // global path rviz publisher (rviz display)
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr globalPathRvizPublisher;

    // global path server
    rclcpp::Service<base_msgs::srv::GlobalPathService>::SharedPtr globalPathServer;

    void responseGlobalPathCallback(const std::shared_ptr<base_msgs::srv::GlobalPathService::Request> request,
                                    const std::shared_ptr<base_msgs::srv::GlobalPathService::Response> response);

    visualization_msgs::msg::Marker path2Marker(const nav_msgs::msg::Path &path);
  };
} // namespace Planning
#endif // ! GLOABAL_PATH_SERVER_H_
