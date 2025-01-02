#ifndef TP_MOVE_CMD_H_
#define TP_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"

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
