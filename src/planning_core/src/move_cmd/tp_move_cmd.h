#ifndef TP_MOVE_CMD_H_
#define TP_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "ego_car_base.h"

namespace Planning {
  class TPMoveCmd : public rclcpp::Node
  {
  public:
    TPMoveCmd();
    TPMoveCmd(const TPMoveCmd &other) = delete;
    TPMoveCmd &operator=(const TPMoveCmd &other) = delete;
    ~TPMoveCmd() = default;

  private:
  };
} // namespace Planning
#endif // ! TP_MOVE_CMD_H_
