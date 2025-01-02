#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning {
  class PNCMapCreatorSTurn : public PNCMapCreatorBase
  {
  public:
    PNCMapCreatorSTurn();
    PNCMapCreatorSTurn(const PNCMapCreatorSTurn&) = delete;
    ~PNCMapCreatorSTurn() = default;

  private:
  };
} // namespace Planning
#endif // ! PNC_MAP_CREATOR_STURN_H_
