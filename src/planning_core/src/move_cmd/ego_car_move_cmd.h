#ifndef EGO_CAR_MOVE_CMD_H_
#define EGO_CAR_MOVE_CMD_H_

#include "rclcpp/rclcpp.hpp"

namespace Planning {
  class EgoCarMoveCmd : public rclcpp::Node
  {
  public:
    EgoCarMoveCmd();
    EgoCarMoveCmd(const EgoCarMoveCmd &other) = delete;
    EgoCarMoveCmd &operator=(const EgoCarMoveCmd &other) = delete;
    ~EgoCarMoveCmd() = default;

  private:
  };
} // namespace Planning
#endif // ! EGO_CAR_MOVE_CMD_H_
