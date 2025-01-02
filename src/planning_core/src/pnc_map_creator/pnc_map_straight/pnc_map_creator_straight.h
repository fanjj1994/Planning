#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_

#include "rclcpp/rclcpp.hpp"
#include "pnc_map_creator_base.h"

namespace Planning
{
    class PNCMapCreatorStraight : public PNCMapCreatorBase
    {
    public:
        PNCMapCreatorStraight();
        PNCMapCreatorStraight(const PNCMapCreatorStraight&) = delete;
        ~PNCMapCreatorStraight() = default;


    private:

    };
}  // namespace Planning
#endif  // ! PNC_MAP_CREATOR_STRAIGHT_H_
