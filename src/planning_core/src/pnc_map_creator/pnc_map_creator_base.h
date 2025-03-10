#ifndef PNC_MAP_CREATOR_BASE_H_
#define PNC_MAP_CREATOR_BASE_H_

#include "rclcpp/rclcpp.hpp"
#include "config_reader.h"
#include "geometry_msgs/msg/point.hpp"
#include "base_msgs/msg/pnc_map.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "common_type.h"

namespace Planning {

  enum class PNCMapType : uint8
  {
    Straight = 0U,
    Turn = 1U,
    STurn = 2U,
    Cross = 3U,
    Road = 4U,
    RoadEnd = 5U,
    RoadStart = 6U,
    RoadCross = 7U,
    RoadTurn = 8U,
    RoadSTurn = 9U,
    RoadCrossTurn = 10U,
    RoadCrossSTurn = 11U,
    RoadCrossTurnSTurn = 12U,
    RoadCrossTurnSTurnEnd = 13U,
    Default = 255U
  };

  class PNCMapCreatorBase
  {
  public:
    PNCMapCreatorBase() = default;
    PNCMapCreatorBase(const PNCMapCreatorBase&) = delete;
    virtual ~PNCMapCreatorBase() = default;

    virtual base_msgs::msg::PNCMap createPNCMap() = 0; // create pnc map

    inline base_msgs::msg::PNCMap getPNCMap() const // get PNC Map
    {
      return pncMap;
    }

    inline visualization_msgs::msg::MarkerArray getPNCMapMarkerArray() const // get PNC Map for visualization
    {
      return pncMapMarkerArray;
    }

  protected:
    std::unique_ptr<ConfigReader> pncMapConfig;
    uint8 mapType{ 0U };
    base_msgs::msg::PNCMap pncMap;
    visualization_msgs::msg::MarkerArray pncMapMarkerArray;
    geometry_msgs::msg::Point pCenter;
    geometry_msgs::msg::Point pLeft;
    geometry_msgs::msg::Point pRight;

    float64 thetaCurrent{ 0.0 };
    float64 lengthStep{ 0.0 };
    float64 thetaStep{ 0.0 };

  private:
  };
} // namespace Planning
#endif // ! PNC_MAP_CREATOR_BASE_H_
