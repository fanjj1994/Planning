#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning {
  class PNCMapCreatorSTurn : public PNCMapCreatorBase
  {
  public:
    PNCMapCreatorSTurn();
    PNCMapCreatorSTurn(const PNCMapCreatorSTurn &) = delete;
    ~PNCMapCreatorSTurn() = default;

    base_msgs::msg::PNCMap createPNCMap() override;

  private:
    void initPNCMap();
    void drawStraightX(const float64 &length, const float64 &plugFlag, const float64 &ratio = 1.0);
    void drawArc(const float64 &angle, const float64 &plusFlag, const float64 &ratio = 1.0);  // draw arc, clockwise -> positive, anticlockwise -> negative
  };
} // namespace Planning
#endif // ! PNC_MAP_CREATOR_STURN_H_
