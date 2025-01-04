#ifndef GLOABAL_PATH_SERVER_H_
#define GLOABAL_PATH_SERVER_H_

#include "rclcpp/rclcpp.hpp"
#include "global_planner_normal.h"

namespace Planning {
  class GlobalPathServer : public rclcpp::Node
  {
  public:
    GlobalPathServer();
    GlobalPathServer(const GlobalPathServer&) = delete;
    ~GlobalPathServer() = default;

  private:
  };
} // namespace Planning
#endif // ! GLOABAL_PATH_SERVER_H_
