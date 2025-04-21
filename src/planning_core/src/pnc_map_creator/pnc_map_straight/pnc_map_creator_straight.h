#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_

#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning {
  class PNCMapCreatorStraight : public PNCMapCreatorBase
  {
  public:
    PNCMapCreatorStraight();
    PNCMapCreatorStraight(const PNCMapCreatorStraight &) = delete;
    ~PNCMapCreatorStraight() = default;

    base_msgs::msg::PNCMap createPNCMap() override;

  private:
    void initPNCMap();
    void drawStraightX(const float64 &length, const float64 &plusFlag, const float64 &ratio = 1.0);
  };
} // namespace Planning
#endif // ! PNC_MAP_CREATOR_STRAIGHT_H_
