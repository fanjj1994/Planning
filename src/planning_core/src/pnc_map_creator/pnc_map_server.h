#ifndef PNC_MAP_SERVER_H_
#define PNC_MAP_SERVER_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning {
  class PNCMapServer : public rclcpp::Node
  {
  public:
    PNCMapServer();
    PNCMapServer(const PNCMapServer&) = delete;
    ~PNCMapServer() = default;

  private:
  };
} // namespace Planning
#endif // ! PNC_MAP_SERVER_H_
